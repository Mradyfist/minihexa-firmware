#include "hiwonder_robot.h"

//initialize the MiniHexa object
Robot minihexa;

//initialize the motion state
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
    vel = {0.0f, 5.0f, 0.2f};//arc forward to the left
    minihexa.move(&vel, &pos, &att);//execute the move
    delay(5000);

    vel = {0.0f, 5.0f, -0.2f};//arc forward to the right
    minihexa.move(&vel, &pos, &att);//execute the move
    delay(5000);
}

