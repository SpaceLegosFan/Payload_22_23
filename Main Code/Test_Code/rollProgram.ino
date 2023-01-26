#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>

#define DELAY_MS (500)

boolean NDOF = true;

float roll;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void) {

  Serial.begin(115200);

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected!");
  }
  delay(100);
  bno.setExtCrystalUse(true);
}

void loop() {
  

  //Request Euler Angles from Sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Request Quaternions from sensor
  imu::Quaternion quat = bno.getQuat();
  
  //Calculate Roll from Quaternions
  float yy = quat.y() * quat.y();

  float roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + yy));

  //Convert to Degrees
  float rollDeg  = 57.2958 * roll;

  if (NDOF) {
    Serial.print("Roll: ");  Serial.print(rollDeg, 2);
    Serial.println(); Serial.println();
  }


  delay(DELAY_MS);
}
