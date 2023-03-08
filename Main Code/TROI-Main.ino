/* 
NDRT Payload 2022-2023
TROI ESP32-Main Code
*/

#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#include "RTClib.h"
#define I2C_SDA 21 // SDA = 21
#define I2C_SCL 22 // SCL = 22
#define I2C_SDA2 32 // SDA2 = 32
#define I2C_SCL2 33 // SCL2 = 33
#define motorInterfaceType 1
#define leadDIR 12 // DIR1 = 26
#define leadSTEP 14 // STEP1 = 25
#define cameraDIR 26  // DIR2 = 13
#define cameraSTEP 25 // STEP2 = 27
#define ACCELERATION_LAND_TOLERANCE .3
#define GYRO_LAND_TOLERANCE 5
#define ACCELERATION_LAUNCH_TOLERANCE 30
#define DEPLOYSTEPS 5200

TwoWire I2CSensors = TwoWire(0);
TwoWire I2CSensors2 = TwoWire(1);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &I2CSensors);
Adafruit_BNO055 bno2 = Adafruit_BNO055(8, BNO055_ADDRESS_A, &I2CSensors2);
AccelStepper LeadScrewStepper(motorInterfaceType, leadSTEP, leadDIR);
AccelStepper CameraStepper(motorInterfaceType, cameraSTEP, cameraDIR);
float cameraAngle = 0.0;

imu::Vector<3> *accelerationQueue = new imu::Vector<3>[10];
imu::Vector<3> *gyroQueue = new imu::Vector<3>[10];
int size = 0;

// Motor Values
float travel_distance = 7.5;
float travel_distance_per_full_step = 0.00125;
float num_deployment_LeadScrew_steps = DEPLOYSTEPS;

// I2C RTC Clock Interface
RTC_DS3231 rtc;

// ESP-NOW - THIS NEEDS TO BE CHANGED, MAC ADDRESS NOT VALID
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x69, 0xD0};
typedef struct struct_message{
  char timestamp[32];
  int command;
} struct_message;
struct_message myData;
esp_now_peer_info_t peerInfo;

char inbyte = 0;                 // Received byte
char buf[260];                   // Incoming data buffer
int buflen = 0;                  // Length of buffered ata
String serialMessage = "";
int beginTime = -1;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(38400);

  // SD Card
  if (!SD.begin()) {
    Serial.println("SD Initialization failed!");
  }
  appendFile(SD, "/payload.txt", "\n\n\nOutput file for payload systems:\n");
  appendFile(SD, "/data.txt", "\n\n\nOutput file for data logging:\n");
  Serial.print("SD Card Initialized.\n");
  appendFile(SD, "/payload.txt", "SD Card Initialized.\n");

  // I2C Sensors
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  I2CSensors2.begin(I2C_SDA2, I2C_SCL2, 100000);
  Wire.begin();
  Serial.println("Initialized both I2CSensors and Wire.");

  // RTC Clock
  if (!rtc.begin(&I2CSensors)) {
    Serial.println("Couldn't find RTC");
  }
  printEvent("RTC Clock Initialized.");

  if (!bno.begin()) {
    printEvent("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  printEvent("BNO is initialized.");
  if (!bno2.begin()) {
    printEvent("Ooops, no BNO055-2 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  printEvent("BNO2 is initialized.");
  bno.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    printEvent("Error initializing ESP-NOW.");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    printEvent("Failed to add peer.");
    return;
  }
  printEvent("ESP-NOW setup finished.");

  // Set stepper motor speeds/accelerations
  LeadScrewStepper.setMaxSpeed(400); // 800
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(400);
  CameraStepper.setMaxSpeed(200);
  CameraStepper.setAcceleration(500);
  CameraStepper.setSpeed(200);

  printEvent("Setup done!");
  printEvent("Standing By for Launch.");

  updateLaunch();
  while (!checkLaunch()) {
    delay(100);
    updateLaunch();
    updateSerialMessage();
    if(serialMessage != ""){
      checkSerialMessage();
    }
  }
  printEvent("We Have Launched!");

  // Wait a minimum of 60 seconds before standing by for landing. Record flight data during this.
  for (int i = 1; i <= 10 * 90; i++) {
    if (i % 100 == 0){
      char timeMessage[50];
      snprintf(timeMessage, 50, "We are %d seconds into flight!", i/10);
      printEvent(timeMessage);
    }
    recordFlightData();
    delay(100);
  }

  // wait in standby mode and loop until landed
  printEvent("Standing By for Landing");
  updateLanding();
  while (!checkLanding()) {
    delay(100);
    updateLanding();
  }
  printEvent("We Have Landed!");
  delay(1000);

  // After landing, check to make sure the payload tube is not rolling
  printEvent("Standing by for checkRoll.");
  checkRoll();
  printEvent("Check Roll Finished.");

  imu::Quaternion q = bno.getQuat();
  float yy = q.y() * q.y();
  float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
  float initialXAngle = 57.2958 * roll;

  char buffer[100];
  int ret = snprintf(buffer, sizeof buffer, "Landed at %f degrees. Standby for horizontal motion.", initialXAngle);
  printEvent(buffer);
  delay(500);

  leadScrewRun();
  printEvent("Done deploying horizontally.");
  delay(2000);

  // deploy vertically
  printEvent("Deploying vertically.");
  spinCameraStepper(-60);
  printEvent("Finished deploying vertically.");
  printEvent("Standing By for Camera commands...");
	serialMessage = "":
}

// standby for RF commands
void loop() {
  while(Serial.available()>0){
    if(beginTime == -1) beginTime = millis();
    int index = Serial.read();
    if((index > 48 && index < 57) || (index > 64 && index < 73)){
      char letter = index;
      serialMessage += letter;
    }
  }
  if(beginTime != -1 && millis() - beginTime > 1000){
    storeEvent("Executing Radio Commands");
    interpretRadioString(serialMessage);
    storeEvent("Done with all radio commands.");
    serialMessage = "";
    beginTime = -1;
  }
}

/*
  Updates the acceleration vectors in the acceleration queue to be used by the checkLaunch() function to check for a launch
*/
void updateLaunch() {
  if (size >= 2) {
    accelerationQueue[0] = accelerationQueue[1];
    size--;
  }
  accelerationQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  size++;
}

/*
  Using the acceleration queue, calculates the average acceleration for the last 10 points
  If the acceleration average is greater than the launch acceleration tolerance, returns true saying the rocket has launched
*/
bool checkLaunch() {
  float accelAverage = 0;
  for (int i = 0; i < size; i++) {
    accelAverage += accelerationQueue[i].magnitude();
  }
  accelAverage /= size;
  storeData("accelAverage", accelAverage);
  return (accelAverage > ACCELERATION_LAUNCH_TOLERANCE == true);
}

/*
  Updates the acceleration and gyro vectors in their respective queues to be used by the checkLanding() function to check for landing.
*/
void updateLanding() {
  if (size >= 10) {
    for (int i = 0; i < 9; i++) {
      accelerationQueue[i] = accelerationQueue[i + 1];
      gyroQueue[i] = gyroQueue[i + 1];
    }
    size--;
  }
  accelerationQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyroQueue[size] = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  size++;
}

/*
  Using the acceleration and gyro queues, calculates the average acceleration and gyroscopic motion for the last 10 points
  If the acceleration and gyro averages are less than their respective landing tolerances, returns true saying the rocket has landed.
*/
bool checkLanding() {
  float accelAverage = 0;
  float gyroAverage = 0;
  for (int i = 0; i < size; i++) {
    accelAverage += accelerationQueue[i].magnitude();
    gyroAverage += gyroQueue[i].magnitude();
  }
  accelAverage /= size;
  gyroAverage /= size;
  accelAverage -= 9.8;
  storeData("accelAverage", accelAverage);
  storeData("gyroAverage", gyroAverage);
  Serial.println(accelAverage);
  Serial.println(gyroAverage);
  Serial.println(accelAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE);
  return (accelAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE);
}

void recordFlightData() {
  updateLanding();
  float accelAverage = 0;
  float gyroAverage = 0;
  for (int i = 0; i < size; i++) {
    accelAverage += accelerationQueue[i].magnitude();
    gyroAverage += gyroQueue[i].magnitude();
  }
  accelAverage /= size;
  gyroAverage /= size;
  storeData("accelAverage", accelAverage);
  storeData("gyroAverage", gyroAverage);
}

void updateSerialMessage(){
  while(Serial.available()>0){
    int index = Serial.read();
    if(index >= 32 && index < 127){
      char letter = index;
      serialMessage += letter;
    }
  }
}

void checkSerialMessage() {
  serialMessage.toLowerCase();
  if (serialMessage != "") {
    if (serialMessage == "run motor")
      leadScrewRun();
    else if (serialMessage == "change direction")
      num_deployment_LeadScrew_steps *= -1;
    else if (serialMessage == "print steps")
      Serial.println(num_deployment_LeadScrew_steps);
    else if(serialMessage == "reset steps")
      num_deployment_LeadScrew_steps = DEPLOYSTEPS;
    else if(serialMessage == "reset motor"){
      int temp = num_deployment_LeadScrew_steps;
      num_deployment_LeadScrew_steps = -DEPLOYSTEPS;
      serialMessage = "run motor";
      num_deployment_LeadScrew_steps = temp;
    }
    else if(serialMessage == "step down"){
      int temp = num_deployment_LeadScrew_steps;
      num_deployment_LeadScrew_steps = -50;
      serialMessage = "run motor";
      num_deployment_LeadScrew_steps = temp;
    }
    else if(serialMessage == "step up"){
      int temp = num_deployment_LeadScrew_steps;
      num_deployment_LeadScrew_steps = 50;
      serialMessage = "run motor";
      num_deployment_LeadScrew_steps = temp;
    }
    else if (serialMessage.indexOf("steps =") != -1)
      num_deployment_LeadScrew_steps = serialMessage.substring(serialMessage.indexOf("=") + 2).toInt();
    else if (serialMessage.indexOf("radio string") != -1)
      interpretRadioString(serialMessage);
    else if (serialMessage.indexOf("camera turn") != -1) {
      int angle = serialMessage.substring(serialMessage.indexOf("=") + 2).toInt();
      spinCameraStepper(angle);
    }
    else if (serialMessage.indexOf("radio command") != -1) {
      int command = serialMessage.substring(serialMessage.indexOf("=") + 2).toInt();
      executeRadioCommand(command);
      Serial.print("The radio command is: ");
      Serial.println(command);
    }
    else
      Serial.println("Not a valid command!");
    serialMessage = "";
  }
}

bool checkRoll() {
  int countCheck = 0;
  float currentRoll = 0;
  float prevRoll = 0;
  while (1) {
    prevRoll = currentRoll;
    imu::Quaternion q = bno.getQuat();
    float yy = q.y() * q.y();
    float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
    float initialXAngle = 57.2958 * roll;
    currentRoll = initialXAngle;

    imu::Quaternion q2 = bno2.getQuat();
    float yy2 = q2.y() * q2.y();
    float roll2 = atan2(2 * (q2.w() * q2.x() + q2.y() * q2.z()), 1 - 2 * (q2.x() * q2.x() + yy2));
    float initialX2Angle = 57.2958 * roll2;
    storeData("roll", roll);
    storeData("roll2", roll2);
    storeData("currentRoll", currentRoll);
    storeData("prevRoll", prevRoll);

    // Within .5 radians of the two BNO055 roll values
    if (roll2 - .5 < roll && roll < roll2 + .5) {
      printEvent("Both IMUs have same data (within 0.5 radians).");
      // Within 10 degrees of the two roll values on the first BNO055
      if (prevRoll - 10 < currentRoll && currentRoll < prevRoll + 10) {
        printEvent("prevRoll is whithin 10 degrees of currentRoll.");
        return true;
      }
    }
    else if (countCheck == 5) { // If a value can not come to a consesus within 5 polls, override the auxillary mechanism
      storeEvent("Both IMU have different data.");
      if (prevRoll - 10 < currentRoll && currentRoll < prevRoll + 10) {
        storeEvent("prevRoll is whithin 10 degrees of currentRoll.");
        return true;
      }
    }
    else if (countCheck != 5)
      countCheck++;
    delay(3000);
  }
}

void leadScrewRun() {
  LeadScrewStepper.move(num_deployment_LeadScrew_steps);
  while (LeadScrewStepper.run()) {}
}

void spinCameraStepper(int angle) {
  if (cameraAngle + angle > 180)
    angle = angle - 360;
  else if (cameraAngle + angle < -180)
    angle = angle + 360;
  Serial.println(angle);
  int steps = angle / 1.8;
  cameraAngle += steps * 1.8;
  Serial.println(steps);
  CameraStepper.move(steps);
  while (CameraStepper.run()) {
  }
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}

void storeEvent(const char *event) {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char *timeString = now.toString(bufferString);
  appendFile(SD, "/payload.txt", timeString);
  appendFile(SD, "/payload.txt", "  -  ");
  appendFile(SD, "/payload.txt", event);
  appendFile(SD, "/payload.txt", "\n");
}

void storeData(const char *type, float data) {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char *timeString = now.toString(bufferString);
  char dataBuffer[64];
  int ret = snprintf(dataBuffer, sizeof dataBuffer, "%f", data);
  appendFile(SD, "/data.txt", timeString);
  appendFile(SD, "/data.txt", "  -  ");
  appendFile(SD, "/data.txt", type);
  appendFile(SD, "/data.txt", "  -  ");
  appendFile(SD, "/data.txt", dataBuffer);
  appendFile(SD, "/data.txt", "\n");
}

void printEvent(const char *event) {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char *timeString = now.toString(bufferString);
  Serial.print(timeString);
  Serial.print(" - ");
  Serial.println(event);
  storeEvent(event);
}

void interpretRadioString(String message) { // "XX4XXX C3 A1 D4 C3 F6 C3 F6 B2 B2 C3."
  storeEvent("Interpreting radio string");
  message.toUpperCase();
  int numberCommands = 0;
  int commands[100];
  while(1){
    int location = findFirstRadioCommand(message);
    if(location == -1) break;
    commands[numberCommands] = message.substring(location+1, location+2).toInt();
    numberCommands++;
    message.remove(location, 2);
  }
  if(numberCommands == 0)
    storeEvent("No commands found in serial message.");
  for (int i = 0; i < numberCommands; i++) {
    executeRadioCommand(commands[i]);
    delay(3000);
  }
}

int findFirstRadioCommand(String message) {
  String possibleCommands[8] = {"A1", "B2", "C3", "D4", "E5", "F6", "G7", "H8"};
  int location = -1;

  for (int i = 0; i < 8; i++) {
    int potentialLocation = message.indexOf(possibleCommands[i]);
    if (potentialLocation != -1 && (location == -1 || potentialLocation < location)) {
      location = potentialLocation;
    }
  }

  return location;
}


void executeRadioCommand(int command) {
  String string = "I am executing command " + String(command);
  printEvent(string);
  switch (command) {
    case 1:
      spinCameraStepper(60);
      break;
    case 2:
      spinCameraStepper(-60);
      break;
    case 3:
      sendData(3);
      delay(12000);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      sendData(command);
      break;
  }
}


void sendData(int commandData) {
  // Get Timestamp
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char *timeString = now.toString(bufferString);

  // Set values to send
  strcpy(myData.timestamp, timeString);
  myData.command = commandData;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK)
    printEvent("Sent with success");
  else
    printEvent("Error sending the data");
}