#include "hiwonder_robot.h"

Robot minihexa;

uint8_t result;

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  result = minihexa.sensor.asr.rec_recognition();
  switch(result) {
    case 1:  /* move forward */
      vel = {0.0f, 2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 2:  /* move backward */
      vel = {0.0f, -2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 3:  /* turn left */
      vel = {0.0f, 0.0f, 2.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 4:  /* turn right */
      vel = {0.0f, 0.0f, -2.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 9:  /* stop */
      vel = {0.0f, 0.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 29:  /* walk two steps */
      vel = {0.0f, 2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att, 1000, 2); 
      delay(2100);
      break;

    default:
      break;
  }
  delay(100);
}
