#include "camera_setting.h"
#include "color_detection.hpp"
#include "iic_data_send.hpp"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueIICData = NULL;

void setup() {
  /* create the image transfer queue */
  xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *)); 
  /* create the IIC data transfer queue */
  xQueueIICData = xQueueCreate(2, sizeof(send_color_data_t));

  /* register the camera processing task */
  register_camera(PIXFORMAT_RGB565, FRAMESIZE_QQVGA, 4, xQueueAIFrame);
  /* register the line-following detection task */
  register_color_detection(xQueueAIFrame, NULL, xQueueIICData, NULL, true);
  /* register the IIC data transfer task */
  register_iic_data_send(xQueueIICData, NULL);

}

void loop() 
{
}
