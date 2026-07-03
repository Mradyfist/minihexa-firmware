#include "camera_setting.h"
#include "face_detection.hpp"
#include "iic_data_send.hpp"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueIICData = NULL;

void setup() 
{
  /* create the image transfer queue */
  xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *)); 
  /* create the IIC data transfer queue */
  xQueueIICData = xQueueCreate(2, sizeof(iic_send_data_t *));

  /* register the camera processing task */
  register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 4, xQueueAIFrame);
  /* register the face detection task */
  register_human_face_detection(xQueueAIFrame, NULL, xQueueIICData, NULL, true);
  /* register the IIC data transfer task */
  register_iic_data_send(xQueueIICData, NULL);
}


void loop() 
{
  
}
