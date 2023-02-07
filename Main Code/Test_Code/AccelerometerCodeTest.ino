// 1 Milligal [mGal] = 0.001 Gal = 0.01 m/s^2
//
//
//
//
// VCC	5V / 3V3
// GND	GND
// SDA	A4(SDA)
// SCL	A5(SCL)

#include <Wire.h>
#include <DFRobot_LIS2DH12.h>

DFRobot_LIS2DH12 LIS;

void setup(){
  Wire.begin();
  Serial.begin(115200);

}

void loop(){
  acceleration();
}

void acceleration(void)
{
  int16_t x, y, z;

  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  float xg = x * 0.01;
  float yg = y * 0.01;
  float zg = z * 0.01;

  float a = sqrt((x^2)+(y^2)+(z^2));
  Serial.print(a);

}
