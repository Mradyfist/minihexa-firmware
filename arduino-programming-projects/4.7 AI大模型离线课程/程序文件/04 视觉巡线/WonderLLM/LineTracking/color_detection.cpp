#include "color_detection.hpp"
#include "esp_log.h"
#include "esp_camera.h"
#include "dl_image.hpp"
#include "fb_gfx.h"
#include "color_detector.hpp"
#include "who_ai_utils.hpp"
#include "camera_setting.h"

using namespace std;
using namespace dl;

static const char *TAG = "color_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gReturnFB = true;

static const size_t segmentHeight = 40;
static const size_t segmentSize = 160 * segmentHeight;
static uint16_t segment1[segmentSize];
static uint16_t segment2[segmentSize];

send_color_data_t block_segment;


/* the user can adjust the color thresholds here
 * structure: color threshold - color area threshold - color name
 */
vector<color_info_t> std_color_info = {
    // { {0, 44, 53, 255, 151, 233}, 64, "red"},
    // {{78, 88, 43, 255, 46, 165}, 64, "green"},
    // {{91, 104, 45, 255, 167, 255}, 64, "blue"},
    // {{125, 155, 70, 255, 90, 255}, 64, "purple"}
    {{138, 255, 48, 255, 211, 255}, 64, "red"},
    {{48, 80, 70, 161, 90, 255}, 64, "green"},
    {{94, 105, 28, 138, 79, 255}, 64, "blue"},
    {{125, 155, 70, 255, 90, 255}, 64, "purple"}
};

static uint8_t state_value;

/* get the color detection result */
static void get_color_detection_result(uint16_t *image_ptr, int image_height, int image_width, 
                                     vector<color_detect_result_t> &results, uint16_t color, 
                                     color_data_t *color_data, int color_index, int segment_index)
{
  int max_color_column_index = 0;
  int local_max_area = 0;
  
  /* find the largest color block of the same color */
  for (int i = 0; i < results.size(); ++i)
  {
    if (results[i].area > local_max_area)
    {
      local_max_area = results[i].area;
      max_color_column_index = i;
    }
  }
  
  // update the data of the current color
  if (results.size() > 0)
  {
    switch (color)
    {
      case COLOR_RED:
        color_data[0].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[0].center_y = (uint8_t)results[max_color_column_index].center[1];
        color_data[0].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        color_data[0].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_GREEN:
        color_data[1].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[1].center_y = (uint8_t)results[max_color_column_index].center[1];
        color_data[1].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        color_data[1].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_BLUE:
        color_data[2].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[2].center_y = (uint8_t)results[max_color_column_index].center[1];
        color_data[2].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        color_data[2].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      case COLOR_PURPLE:
        color_data[3].center_x = (uint8_t)results[max_color_column_index].center[0];
        color_data[3].center_y = (uint8_t)results[max_color_column_index].center[1];
        color_data[3].width = (uint8_t)(results[max_color_column_index].box[2] - results[max_color_column_index].box[0]);
        color_data[3].length = (uint8_t)(results[max_color_column_index].box[3] - results[max_color_column_index].box[1]);
        break;

      default:
        break;
    }
  }
}

static void modifyPixel(uint16_t *imageBuffer, size_t width, 
                        size_t height, size_t x, size_t y, 
                        uint8_t newRed, uint8_t newGreen, uint8_t newBlue) {

    if (x >= width || y >= height) {
        // std::cerr << "Pixel coordinates out of bounds" << std::endl;
        return;
    }

    // compute the pixel index in the one-dimensional array
    int index = y * width + x;

    // combine the new color value
    uint16_t newPixel = (newRed << 11) | (newGreen << 5) | newBlue;

    // write the modified pixel value back to the image buffer
    imageBuffer[index] = newPixel;
}

// function to draw a rectangle box
static void drawRectangle(uint16_t *imageBuffer, size_t width, size_t height, 
                         int x, int y, int w, int h, uint16_t color) {
    // ensure the coordinates are within the valid range
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + w >= width) w = width - x - 1;
    if (y + h >= height) h = height - y - 1;
    
    // draw the top and bottom edges
    for (int i = x; i <= x + w; i++) {
        if (i < width && y < height) {
            imageBuffer[y * width + i] = color;
        }
        if (i < width && (y + h) < height) {
            imageBuffer[(y + h) * width + i] = color;
        }
    }
    
    // draw the left and right edges
    for (int i = y; i <= y + h; i++) {
        if (x < width && i < height) {
            imageBuffer[i * width + x] = color;
        }
        if ((x + w) < width && i < height) {
            imageBuffer[i * width + (x + w)] = color;
        }
    }
}

// draw the detected color boxes on the image (each segment drawn independently)
static void drawColorBoxes(uint16_t *imageBuffer, size_t width, size_t height, 
                          color_data_t *segment1, color_data_t *segment2) {
    // define the box color for each color
    uint16_t colorBoxes[] = {
        COLOR_RED,    // red box
        COLOR_GREEN,  // green box
        COLOR_BLUE,   // blue box
        COLOR_PURPLE  // purple box
    };
    
    // draw the segment1 box (upper half)
    for (int i = 0; i < COLOR_NUM; i++) {
        if (segment1[i].width > 0 && segment1[i].length > 0) {
            // compute the coordinates in the full image
            int x = segment1[i].center_x - segment1[i].width / 2;
            int y = segment1[i].center_y - segment1[i].length / 2;
            
            // ensure the coordinates are within segment1
            if (x >= 0 && y >= 0 && x + segment1[i].width < width && y + segment1[i].length < height / 3) {
                // draw the rectangle box
                drawRectangle(imageBuffer, width, height, x, y, 
                             segment1[i].width, segment1[i].length, colorBoxes[i]);
            }
        }
    }
    
    // draw the segment2 box (middle half)
    for (int i = 0; i < COLOR_NUM; i++) {
        if (segment2[i].width > 0 && segment2[i].length > 0) {
            // compute the coordinates in the full image (must add segment1's height offset)
            int x = segment2[i].center_x - segment2[i].width / 2;
            int y = segment2[i].center_y - segment2[i].length / 2 + (height / 3);
            
            // ensure the coordinates are within segment2
            if (x >= 0 && y >= height / 3 && x + segment2[i].width < width && y + segment2[i].length < 2 * height / 3) {
                // draw the rectangle box
                drawRectangle(imageBuffer, width, height, x, y, 
                             segment2[i].width, segment2[i].length, colorBoxes[i]);
            }
        }
    }
}

// split the RGB565 image into three parts
static void splitImageIntoThreeSegments(uint16_t *imageBuffer, size_t width, size_t height, 
                                 uint16_t *segment1, uint16_t *segment2) {
    size_t segmentHeight = height / 3;
    size_t segmentSize = width * segmentHeight;

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            int index = (int)(y * width + x);
            int localIndex = (int)((y % segmentHeight) * width + x);
            if (y < segmentHeight) {
                // part one
                segment1[localIndex] = imageBuffer[index];
            } 
            else if (y < 2 * segmentHeight) {
                // part two
                segment2[localIndex] = imageBuffer[index];
            } 
        }
    }
}

static void mergeSegmentsIntoImage(uint16_t *newImage, size_t width, size_t height, 
                            uint16_t *segment1, uint16_t *segment2) {
    size_t segmentHeight = height / 3;
    size_t segmentSize = width * segmentHeight;

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            int index = y * width + x;
            int localIndex = (y % segmentHeight) * width + x;

            if (y < segmentHeight) {
                // part one
                newImage[index] = segment1[localIndex];
            } else if (y < 2 * segmentHeight) {
                // part two
                newImage[index] = segment2[localIndex];
            }

        }
    }
}

static void task_process_handler(void *arg)
{
  camera_fb_t *frame = NULL;
  ColorDetector detector;
  uint32_t processed_frames = 0;

  /* register color information */
  for (int i = 0; i < std_color_info.size(); ++i)
  {
    detector.register_color(std_color_info[i].color_thresh, std_color_info[i].area_thresh, std_color_info[i].name);
  }
  vector<uint16_t> draw_colors = {
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
  };
  int draw_colors_num = draw_colors.size();
  while (true)
  {
    if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
    {
      processed_frames++;
      if (processed_frames % 100 == 0) {
        ESP_LOGI(TAG, "Processed %d frames", processed_frames);
      }
      
      splitImageIntoThreeSegments((uint16_t *)frame->buf, frame->width, frame->height, segment1, segment2);

      std::vector<std::vector<color_detect_result_t>> &results1 = detector.detect(segment1,{(int)30, (int)frame->width, 3});
      for(int i = 0; i < COLOR_NUM; ++i)
      {
        if(results1[i].size() == 0)
        {
          block_segment.segment1[i].center_x = 0;
          block_segment.segment1[i].center_y = 0;
          block_segment.segment1[i].width = 0;
          block_segment.segment1[i].length = 0;
        }
      }
      
      for (int i = 0; i < results1.size(); ++i)
      {
        get_color_detection_result((uint16_t *)segment1, (int)30, (int)frame->width, results1[i], 
                                 draw_colors[i % draw_colors_num], block_segment.segment1, i, 0);
      }

      std::vector<std::vector<color_detect_result_t>> &results2 = detector.detect(segment2,{(int)30, (int)frame->width, 3});
      for (int i = 0; i < COLOR_NUM; ++i)
      {
        if(results2[i].size() == 0)
        {
          block_segment.segment2[i].center_x = 0;
          block_segment.segment2[i].center_y = 0;
          block_segment.segment2[i].width = 0;
          block_segment.segment2[i].length = 0;
        }
      }    
      for (int i = 0; i < results2.size(); ++i)
      {
        get_color_detection_result((uint16_t *)segment2, (int)30, (int)frame->width, results2[i], 
                                 draw_colors[i % draw_colors_num], block_segment.segment2, i, 1);
      }  

      mergeSegmentsIntoImage((uint16_t *)frame->buf, frame->width, frame->height, segment1, segment2);
      
      // draw the detected color box on the image
      drawColorBoxes((uint16_t *)frame->buf, frame->width, frame->height, 
                    block_segment.segment1, block_segment.segment2);
      
      tft_show_rgb565((const uint16_t *)frame->buf, frame->width, frame->height);
      
      // release the frame buffer immediately to avoid buffer buildup
      esp_camera_fb_return(frame);
      frame = NULL;
    }
    
    // send the detection result to the IIC queue
    if (xQueueResult)
    {
      xQueueSend(xQueueResult, &block_segment, portMAX_DELAY);             
    }          
  }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
    }
}

void register_color_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
  xQueueFrameI = frame_i;
  xQueueFrameO = frame_o;
  xQueueEvent = event;
  xQueueResult = result;
  gReturnFB = camera_fb_return;

  xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

}