/* TROI ESP-Main Code
NDRT Payload 2022-2023
**ARDUINO**
*/

// Pins for full-scale lead screw (color of the heat shrinks)
// B2 YELLLOW
// A2 WHITE
// A1 BLUE
// B1 RED

#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#include "RTClib.h"
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_SDA2 32
#define I2C_SCL2 33
#define motorInterfaceType 1
#define leadDIR 12
#define leadSTEP 14
#define cameraDIR 12 // 34
#define cameraSTEP 14 // 35
#define ACCELERATION_LAND_TOLERANCE .3
#define GYRO_LAND_TOLERANCE 5
#define ACCELERATION_LAUNCH_TOLERANCE 30

void appendFile(fs::FS &fs, const char * path, const char * message);

TwoWire I2CSensors = TwoWire(0);
TwoWire I2CSensors2 = TwoWire(1);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &I2CSensors);
Adafruit_BNO055 bno2 = Adafruit_BNO055(8, BNO055_ADDRESS_A, &I2CSensors2);

imu::Vector<3>* accelerationQueue;
imu::Vector<3>* gyroQueue;
int size;

AccelStepper LeadScrewStepper(motorInterfaceType, leadSTEP, leadDIR);
AccelStepper CameraStepper(motorInterfaceType, cameraSTEP, cameraDIR);
float cameraAngle;

// Motor Values
float travel_distance = 8.6; // Ask Spencer or https://drive.google.com/drive/u/0/folders/1Yd59MVs0kGjNgtfuYpVg5CDFZwnHGlRj.
float num_steps = 400; // Steps per rotation; this would be if we are half-stepping (units: steps/revolution).
float travel_distance_per_full_step = 0.00125; // Inches per step.
float num_deployment_LeadScrew_steps = travel_distance / travel_distance_per_full_step;

// I2C RTC Clock Interface
RTC_DS3231 rtc;

// WiFi
AsyncWebServer server(80);
const char* ssid = "\x48\x75\x6e\x74\x65\x72\xe2\x80\x99\x73\x20\x69\x50\x68\x6f\x6e\x65"; // Your WiFi SSID
const char* password = "hunter123";  // WiFi Password
const char* ssid_backup = "ND-guest";
const char* password_backup = "";

// ESP-NOW - THIS NEEDS TO BE CHANGED, MAC ADDRESS NOT VALID
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x9D, 0x46, 0x40};
typedef struct struct_message {
  char timestamp[32];
  int command;
} struct_message;
struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  size = 0;
  cameraAngle = 0.0;
  accelerationQueue = new imu::Vector<3>[10];
  gyroQueue = new imu::Vector<3>[10];
  Serial.begin(115200);

  // Wifi setup. Accessible at "<IP Address>/webserial" in browser
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(500);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    WiFi.begin(ssid_backup, password_backup);
    delay(500);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi backup failed!\n");
    }
    else
      Serial.print("WiFi backup initialized");
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  WebSerialPro.begin(&server);
  WebSerialPro.msgCallback(recvMsg);
  server.begin();
  Serial.println("WiFi setup finished.");


  // SD Card
  if (!SD.begin()) {
    Serial.println("SD Initialization failed!");
  }
  appendFile(SD, "/payload.txt", "\n\n\nOutput file for payload systems:\n");
  appendFile(SD, "/data.txt", "\n\n\nOutput file for data logging:\n");
  Serial.print("SD Card Initialized.\n");
  WebSerialPro.println("SD Card Initialized.");
  appendFile(SD, "/payload.txt", "SD Card Initialized.\n");

  // RTC Clock
  if (! rtc.begin(&I2CSensors)) {
    Serial.println("Couldn't find RTC");
  }
  printEvent("RTC Clock Initialized.");

  // I2C Sensors
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  I2CSensors2.begin(I2C_SDA2, I2C_SCL2, 100000);
  Wire.begin();


  printEvent("Initialized all three TwoWire objects.");
  if (!bno.begin()) {
    printEvent("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  printEvent("BNO is initialized.\n");
  if (!bno2.begin()) {
    printEvent("Ooops, no BNO055-2 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  printEvent("BNO2 is initialized.\n");
  bno.setExtCrystalUse(true);



  // ESP-NOW setup
  if (esp_now_init() != ESP_OK) {
    printEvent("Error initializing ESP-NOW.");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  // Register peer
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
  delay(500);
  printEvent("Standing By for Launch.");
 

  bool standby = true;
  while (standby == true) {
    updateLaunch();
    standby = !checkLaunch();
    if (standby == false) {
      delay(100);
    }
  }
  printEvent("We Have Launched!");

  // Wait a minimum of 60 seconds before standing by for landing. Record flight data during this.
  for(int i = 0; i = 10 * 60; i++) {
    recordFlightData();
    delay(100);
  }


  // wait in standby mode and loop until landed
  printEvent("Standing By for Landing");
  standby=true;
  while(standby == true) {
    updateLanding();
    standby = !checkLanding();
    if(standby == false) {
      delay(100);
    }
  }
  printEvent("We Have Landed!");
  delay(1000);

  // After landing, check to make sure the payload tube is not rolling
  checkRoll();
  printEvent("Check Roll Finished.");

  // 
  imu::Quaternion q = bno.getQuat();
  float yy = q.y() * q.y();
  float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
  float initialXAngle = 57.2958 * roll;

  char buffer[64];
  int ret = snprintf(buffer, sizeof buffer, "%f", initialXAngle);
  printEvent("Landed at ");
  printEvent(buffer);
  printEvent(" degrees. Standby for horizontal motion.");
  delay(500);

  leadScrewRun();
  printEvent("Done deploying horizontally.");
  delay(2000);

  // deploy vertically
  printEvent("Deploying vertically.");
  spinCameraStepper(30);

  printEvent("Finished deploying vertically.");
  printEvent("Standing By for Camera commands...");
}


// standby for RF commands
void loop() {
}


void recvMsg(uint8_t *data, size_t len) {
  WebSerialPro.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerialPro.println("Here");
  Serial.println("Here");
  WebSerialPro.println(d);
  d.toLowerCase();
  if(d == "run motor")
    leadScrewRun();
  else if(d == "change direction")
    changeStepperDirection();
  else if(d.indexOf("radio string") != -1) {
    String radioMessage = d.substring(d.indexOf("=") + 2);
    WebSerialPro.print("The radio message is: ");
    WebSerialPro.println(radioMessage);
    interpretRadioString(radioMessage);
  }
  else if(d.indexOf("camera turn") != -1) {
    int angle = d.substring(d.indexOf("=") + 2).toInt();
    WebSerialPro.print("The camera motor will turn ");
    WebSerialPro.print(angle);
    WebSerialPro.println(" degrees.");
    spinCameraStepper(angle);
  }
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
  Updates the acceleration and gyro vectors in their respective queues to be used by the checkLanding() function to check for landing.
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
  If the acceleration and gyro averages are less than their respective landing tolerances, returns true saying the rocket has landed.
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
  gyroAverage -= 9.8;
  if (accelerationAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE) {
    return true;
  } else {
    return false;
  }
}

void recordFlightData() {
  updateLanding();
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
    if ( roll2 - .5 < roll && roll < roll2 + .5) {
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
    else if (countCheck != 5) {      
      countCheck++;
    }
  delay(3000);
  }
}

void changeStepperDirection() {
  num_deployment_LeadScrew_steps *= -1;
}

void leadScrewRun() {
  LeadScrewStepper.move(num_deployment_LeadScrew_steps);
  while(LeadScrewStepper.run()) {}
}

void spinCameraStepper(int angle) {
  if(cameraAngle + angle > 180) {
    angle = 360 - angle;
  }
  else if(cameraAngle + angle < -180) {
    angle = angle + 360;
  }
  int steps = angle/1.8;
  cameraAngle += steps*1.8;
  CameraStepper.move(steps);
  while(CameraStepper.run()) {}
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
  Serial.println("Failed to open file for writing");
  return;
  }
if(!file.print(message)) {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char * path) {
  if(!fs.remove(path)) {
    Serial.println("Delete failed");
  }
}

void printTime() {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);
  Serial.print(timeString);
  Serial.print(" - ");
  WebSerialPro.print(timeString);
  WebSerialPro.print(" - ");
}

void storeEvent(const char* event) {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);
  appendFile(SD, "/payload.txt", timeString);
  appendFile(SD, "/payload.txt", "  -  ");
  appendFile(SD, "/payload.txt", event);
  appendFile(SD, "/payload.txt", "\n");
}

void storeData(const char* type, float data) {
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

void printEvent(const char* event) {
  printTime();  
  Serial.println(event);
  WebSerialPro.println(event);
  storeEvent(event);
}

void interpretRadioString(String message) { // "XX4XXX C3 A1 D4 C3 F6 C3 F6 B2 B2 C3."
  message.toUpperCase();
  message = message.substring(6);
  int numberCommands = message.length()/3;
  int commands[numberCommands];
  for(int i = 0; i < numberCommands; i++) {
    message = message.substring(1);
    commands[i] = message.substring(1,2).toInt();
    message = message.substring(2);
  }
  for(int i = 0; i < numberCommands; i++) {
    switch(commands[i]) {
      case 1:
        spinCameraStepper(60);
        break;
      case 2:
        spinCameraStepper(-60);
        break;
      case 3:
        sendData(3);
        break;
      case 4:
        sendData(4);
        break;
      case 5:
        sendData(5);
        break;
      case 6:
        sendData(6);
        break;
      case 7:
        sendData(7);
        break;
      case 8:
        sendData(8);
        break;
    }
  }
}

void sendData(int commandData) {
  // Get Timestamp
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char* timeString = now.toString(bufferString);

  // Set values to send
  strcpy(myData.timestamp, timeString);

  // Dummy test; sends random "commands"
  myData.command = commandData;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    printEvent("Sent with success");
  }
  else {
    printEvent("Error sending the data");
  }
}
