/* Preliminary Accelerometer Code 
NDRT Payload 2022-2023

Reference Example: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
https://wiki.dfrobot.com/BNO055_Intelligent_9_Axis_Sensor_Module_SKU_SEN0374

CHECK CODE IN ADAFRUIT_BNO055.H LIBRARY
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
