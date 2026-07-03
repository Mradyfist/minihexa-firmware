#include "hiwonder_robot.h"
//initialize the minihexa object
Robot minihexa;

//define the motion mode variable count
uint8_t count = 0;
//number of iterations in the discretization process (number of footfalls)
int step_num = -1;
//initialize the motion time
uint32_t move_time = 1000;
//leg lift height
float leg_lift = 2.0f;
//initialize the body motion
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0:
      count++;
      vel = {0.0f, 2.0f, 0.0f};//move forward
      move_time = 600;//define the run time
      step_num = 3;
      break;
  
    case 1:
      count++;
      vel = {0.0f, 2.0f, 0.0f};
      move_time = 1000;
      step_num = 2;
      break;

    case 2:
      count++;
      vel = {0.0f, -2.0f, 0.0f};
      move_time = 600;
      step_num = 3;
      break;

    case 3:
      count++;
      vel = {0.0f, -2.0f, 0.0f};
      move_time = 1000;
      step_num = 2;
      break;

    case 4:
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      move_time = 600;
      step_num = 2;
      break;

    case 5:
      vel = {0.0f, 0.0f, -2.0f};
      move_time = 1000;
      step_num = -1;
      break;
  }

  minihexa.move(&vel, &pos, &att, move_time, step_num);
  delay(4000);
}
