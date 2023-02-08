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



// Pins for pre-prototyping lead screw
//B2 RED.  
//A2 BLUE
//A1 GREEN
//B1 BLACK
//

// Pins for full-scale lead screw (color of the heat shrinks)
//B2 YELLLOW
//A2 WHITE
//A1 BLUE
//B1 RED



//#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "FS.h"
#include "SD_MMC.h"
#include "SD.h"
#include "SPI.h"
#include "RTClib.h"

//TwoWire I2CSensors = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(/*-1, BNO055_ADDRESS_A, &I2CSensors*/);
Adafruit_BNO055 bno2 = Adafruit_BNO055(/*-1, BNO055_ADDRESS_A, &I2CSensors*/);


imu::Vector<3>* accelerationQueue;
imu::Vector<3>* gyroQueue;
int size;
const float ACCELERATION_LAND_TOLERANCE = .3;
const float GYRO_LAND_TOLERANCE = 5;
const float ACCELERATION_LAUNCH_TOLERANCE = 20;
const int TIME_BETWEEN_UPDATES = 100; // time in ms

// I2C RTC Clock Interface
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


void setup(){
  size = 0;
  accelerationQueue = new imu::Vector<3>[10];
  gyroQueue = new imu::Vector<3>[10];
  Serial.begin(115200);
  //pinMode(SD_CS_PIN, OUTPUT); // SS
  //SPIClass spiSD = SPIClass(HSPI); // Neither HSPI nor VSPI seem to work
  //spiSD.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN); 
  if (!SD.begin()) {
    Serial.println("SD Initialization failed!");
    return;
  }
  writeFile(SD, "/payload.txt", "Output file for payload systems:\n");
  writeFile(SD, "/data.txt", "Output file for data logging:\n");
  Serial.print("SD Card Initialized.\n");
  //storeEvent("SD Card Initialized.");
  //I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.begin();
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
    return;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  //Initalize Clock
  if (! rtc.begin()) 
  {
  Serial.println("Couldn't find RTC");
  } 
  rtc.adjust(DateTime(__DATE__, __TIME__));
  

  Serial.print("Setup done\n");
  getTime();
  storeEvent("Setup done");
  delay(1000);
  Serial.print("Standing By for Launch\n");
  storeEvent("Standing By for Launch");
  getTime();
  
  
 


  bool standby = true;
  while (standby == true)
  {
    updateLaunch();
    standby = !checkLaunch();
    if (standby == false){
      delay(TIME_BETWEEN_UPDATES);
    }
    delay(100);
  }
  Serial.print("We Have Launched!\n");
  storeEvent("We Have Launched!");
  getTime();

  

  delay(5000);


  // wait in standby mode and loop until landed
  Serial.print("Standing By for Landing\n");
  getTime();
  storeEvent("Standing By for Landing");
  standby=true;
  while(standby == true){
    updateLanding();

    standby = !checkLanding;
    if(standby == false){
      delay(TIME_BETWEEN_UPDATES);
    }
  }
  Serial.print("We Have Landed!\n");
  storeEvent("We Have Landed!");
  getTime();
  delay(1000);
  




  // After landing, deploy camera assembly horizontally


  if  (checkRoll() == true)
  {

  imu::Quaternion q = bno.getQuat();
  float yy = q.y() * q.y();
  float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
  float initialXAngle = 57.2958 * roll;

  char buffer[64];
  int ret = snprintf(buffer, sizeof buffer, "%f", initialXAngle);

  Serial.print("Landed at ");
  Serial.print(initialXAngle);
  Serial.print(" degrees. Standby for horizontal motion.\n");
  storeEvent("Landed at ");
  storeEvent(buffer);
  storeEvent(" degrees. Standby for horizontal motion.");



  delay(1000);

  }



  //LeadScrewStepper.run();
  Serial.print("Done deploying Horizontally. Standby for camera orientation.\n");
  storeEvent("Done deploying Horizontally. Standby for camera orientation.");

  //LeadScrewStepper.moveTo(-initialXAngle/1.8);
  delay(1000);



  // Orientation
  Serial.print("Orienting\n");
  appendFile(SD, "/payload.txt", "Orientating\n");
  
  //LeadScrewStepper.run();

  Serial.print("Orientation Complete\n");
  appendFile(SD, "/payload.txt", "Orientation Complete\n");
  delay(1000);


  // deploy vertically
  Serial.print("Deploying Vertically\n");
  appendFile(SD, "/payload.txt", "Deploying Vertically\n");
  /*

    Code to deploy vertically goes here

  */
  delay(5000);
  Serial.print("Standing By for Camera commands...\n");
  appendFile(SD, "/payload.txt", "Standing By for Camera commands...\n");
}



// standby for RF commands
void loop() {
}



/*
  Updates the acceleration vectors in the acceleration queue to be used by the checkLaunch() function to check for a launch
*/
void updateLaunch() {
  if (size >= 2) {
    for (int i = 0; i < 1; i++) {
      accelerationQueue[i] = accelerationQueue[i+1];
    }
    size--;
  }
  accelerationQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  size++;
}




/* 
  Using the acceleration queue, calculates the average acceleration for the last 10 points
  If the acceleration average is greater than the launch acceleration tolerance, returns true saying the rocket has launched

Question: Why do we need to know when the rocket has launched?
*/
bool checkLaunch() {
  float a_avg = 0;
  for (int i = 0; i < size; i++) {
    a_avg += accelerationQueue[i].magnitude();
  }
  a_avg /= size;
  storeData("a_avg", a_avg);
  if (a_avg > ACCELERATION_LAUNCH_TOLERANCE == true) {
    return true;
  } else {
    return false;
  }
}




/*
  Updates the acceleration and gyro vectors in their respective queues to be used by the checkLanding() function to check for landing
*/
void updateLanding() {
  if (size >= 10) {
    for (int i = 0; i < 9; i++) {
      accelerationQueue[i] = accelerationQueue[i+1];
      gyroQueue[i] = gyroQueue[i+1];
    }
    size--;
  }
  accelerationQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyroQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  size++;
}




/* 
  Using the acceleration and gyro queues, calculates the average acceleration and gyroscopic motion for the last 10 points
  If the acceleration and gyro averages are less than their respective landing tolerances, returns true saying the rocket has landed
  
Question: Is this a calculation of linear acceleration or IMU acceleration, we may need to convert it (or does it matter all things being equal?)?
How do we determine between real linear acceleration and Coriolis linear acceleration? Does that matter due to bouncing issues?
*/
bool checkLanding() {
  float accelerationAverage = 0;
  float gyroAverage = 0;
  for (int i = 0; i < size; i++) {
    accelerationAverage += accelerationQueue[i].magnitude();
    gyroAverage += gyroQueue[i].magnitude();
  }
  accelerationAverage /= size;
  gyroAverage /= size;
  storeData("accelerationAverage", accelerationAverage);
  storeData("gyroAverage", gyroAverage);
  if (accelerationAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE) {
    return true;
  } else {
    return false;
  }
}

bool checkRoll() {

while (true) {
  float currentRoll = 0;
  float prevRoll = 0;
  currentRoll = prevRoll;
  imu::Quaternion q = bno.getQuat();
  float yy = q.y() * q.y();
  float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
  float initialXAngle = 57.2958 * roll;
  currentRoll = initialXAngle;

  storeData("currentRoll", currentRoll);

  if (prevRoll * 0.9 < currentRoll && currentRoll < prevRoll * 1.1) {
    return true;
  } 
  delay(3000);
}
}




void createDir(fs::FS &fs, const char * path){
  if(!fs.mkdir(path)){
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  if(!fs.rmdir(path)){
    Serial.println("rmdir failed");
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_WRITE);
  if(!file){
  Serial.println("Failed to open file for writing");
  return;
  }
if(!file.print(message)){
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(!file.print(message)){
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  if (!fs.rename(path1, path2)) {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  if(!fs.remove(path)){
    Serial.println("Delete failed");
  }
}

void getTime(){
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);
  Serial.println(timeString);
}

void storeEvent(char* event){
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);
  appendFile(SD, "/payload.txt", timeString);
  appendFile(SD, "/payload.txt", "  -  ");
  appendFile(SD, "/payload.txt", event);
  appendFile(SD, "/payload.txt", "\n");
}

void storeData(char* type, float data){
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);
  char dataBuffer[64];
  int ret = snprintf(dataBuffer, sizeof dataBuffer, "%f", data);
  appendFile(SD, "/data.txt", timeString);
  appendFile(SD, "/data.txt", "  -  ");
  appendFile(SD, "/data.txt", type);
  appendFile(SD, "/data.txt", "  -  ");
  appendFile(SD, "/data.txt", dataBuffer);
  appendFile(SD, "/data.txt", "\n");
}
