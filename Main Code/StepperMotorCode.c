/* Preliminary Stepper Motor Code
NDRT Payload 2022-2023
**ARDUINO**

Reference Example: https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/

Other helpful websites: https://hackaday.io/project/183713/instructions
https://hackaday.io/project/183279-accelstepper-the-missing-manual
***********
***ESP32***

Add ESP32 to Arduino IDE: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

Reference Example: https://microcontrollerslab.com/stepper-motor-a4988-driver-module-esp32/
*********** */

// this code was copied from the ESP32 reference example: https://microcontrollerslab.com/stepper-motor-a4988-driver-module-esp32/
#include <AccelStepper.h>
#include <AccelerometerCode.c>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// this depends on the GPIO pins we solder
const int DIR = 12;
const int STEP = 14;

const int steps_per_rev = 200;

#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP, DIR);

int main()
{
  // set up the motor...duh
  setup();

  // loop sensor data until deployment is needed
  standyBy();

  deployHor();
  orient();
  deployVert();

  // TODO: call RF functions
}

void setup()
{
  Serial.begin(115200);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(60);
  motor.setSpeed(200);

  // this is the distance the motor will go, not to critical but needs to be tested
  motor.moveTo(200);
}

void standBy()
{
  // TODO iterate data to determine state

  return
}

int deployHor()
{
  // TODO: run motor for predetermined time until camera module exits rocket
  motor.runToPosition();
  return 0;
}

int orient()
{
  // TODO: loop run until camera is vertical
  while (...)
  {
    motor.move(1);
    motor.run();
  }
}

int deployVert()
{
}
