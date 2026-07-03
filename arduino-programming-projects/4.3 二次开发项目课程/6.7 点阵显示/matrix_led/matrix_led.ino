#include "hiwonder_robot.h"
#include "hiwonder_sensor.h"
#include "WMMatrixLED.h"

//create the minihexa object
Robot minihexa;
//initialize the dot-matrix module pins
WMMatrixLed matrix(14, 32);  //  SCK / DIN pin numbers

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  matrix.setBrightness(5);//set brightness
  matrix.clearScreen();//clear the screen
}
int x;
void loop() {
  const char* text = "Hiwonder";
  int textWidth = 6 * strlen(text); // each character is about 6 pixels
  for (int x = 16; x > -textWidth; x--) {
    matrix.drawStr(x, 8, text);       //start displaying
    delay(100);
  }
}
