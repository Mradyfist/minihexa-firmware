#include "hiwonder_robot.h"

//initialize the MiniHexa object
Robot minihexa;

//define the variable count to record the action mode
uint8_t count = 0;
//initialize the motion state
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0://move forward
      count++;
      vel = {0.0f, 3.0f, 0.0f};
      break;
  
    case 1://move forward to the right
      count++;
      vel = {2.0f, 2.0f, 0.0f};
      break;

    case 2://move right
      count++;
      vel = {3.0f, 0.0f, 0.0f};
      break;

    case 3://move backward to the right
      count++;
      vel = {2.0f, -2.0f, 0.0f};
      break;

    case 4://move backward
      count++;
      vel = {0.0f, -3.0f, 0.0f};
      break;

    case 5://move backward to the left
      count++;
      vel = {-2.0f, -2.0f, 0.0f};
      break;

    case 6://move left
      count++;
      vel = {-3.0f, 0.0f, 0.0};
      break;

    case 7://move forward to the left
      count++;
      vel = {-2.0f, 2.0f, 0.0f};
      break;

    case 8://turn left in place
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      break;

    case 9://turn right in place
      count = 0;
      vel = {0.0f, 0.0f, -2.0f};
      break;
  }
  delay(5500);
  minihexa.move(&vel, &pos, &att, 1800, 3);
}
