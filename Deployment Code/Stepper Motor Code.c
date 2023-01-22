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

//this depends on the DPIO pins we solder
const int DIR = 12;
const int STEP = 14;

onst int  steps_per_rev = 200;

#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP, DIR);

void setup() {
  Serial.begin(115200);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(60);
  motor.setSpeed(200);
  motor.moveTo(200);
}

void loop() {
  if (motor.distanceToGo() == 0) {
    motor.moveTo(-motor.currentPosition());
    Serial.println("Rotating Motor in opposite direction...");
  }
  motor.run();
}

int deployHor(){
//TODO: run motor for predetermined time until camera module exits rocket


}

int deployVer(){
  //TODO: run linear actuator until camera is fully extended above horizon
}
