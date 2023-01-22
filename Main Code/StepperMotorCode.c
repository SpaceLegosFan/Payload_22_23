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
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
//  this code was copied from the ESP32 reference example: https://microcontrollerslab.com/stepper-motor-a4988-driver-module-esp32/
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
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{
  Serial.begin(115200);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(60);
  motor.setSpeed(200);
  // this is the distance the motor will go, not to critical but needs to be tested
  motor.moveTo(200);

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.print("Setup done");

  delay(1000);

  // wait in standby mode and loop until landed
  Serial.print("Standing By");
  while (...)
  {
  }

  delay(1000);
  // After landing deploy horizontally
  Serial.print("Deploying Horizontlally");
  motor.runToPosition();

  delay(1000);
  // orient
  Serial.print("Orienting");
  sensors_event_t event;
  bno.getEvent(&event);
  while ((event.orientation.x - verticalValue) > 10)
  {
    // Serial.print("Orientation: ");
    // Serial.print(event.orientation.x, 4);
    motor.move(1);
    motor.run();
    bno.getEvent(&event);
  }

  Serial.print("Orientation Complete");

  delay(1000);

  // deploy vertically
  Serial.print("Deploying Vertically");

  Serial.print("Standing By for Camera commands...");
}

// standby for RF commands
void loop()
{
}