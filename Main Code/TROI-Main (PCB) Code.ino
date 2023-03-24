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
#include <EEPROM.h>
#include <WiFi.h>
#include "FS.h"
#define EEPROM_SIZE 512
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_SDA2 32
#define I2C_SCL2 33
#define motorInterfaceType 1
#define leadDIR 26
#define leadSTEP 25
#define cameraDIR 13
#define cameraSTEP 27
#define ACCELERATION_LAND_TOLERANCE .3
#define GYRO_LAND_TOLERANCE 5
#define ACCELERATION_LAUNCH_TOLERANCE 30
#define DEPLOYSTEPS 5700

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

// Data Storage
int address = 0;

// Motor Values
float num_deployment_LeadScrew_steps = DEPLOYSTEPS;

// ESP-NOW - THIS NEEDS TO BE CHANGED, MAC ADDRESS CURRENTLY VALID
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x9D, 0x46, 0x40};
typedef struct{
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

int commands[100];

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(38400);

  // Initialize EEPROM with predefined size
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM...");
    ESP.restart();
  } else {
    Serial.println("Success to initialise EEPROM...");
  }

  // I2C Sensors
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  I2CSensors2.begin(I2C_SDA2, I2C_SCL2, 100000);
  Wire.begin();
  Serial.println("Initialized both I2CSensors and Wire.");

  if (!bno.begin()) {
    printEvent("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    ESP.restart();
  }
  printEvent("BNO is initialized.");
  if (!bno2.begin()) {
    printEvent("Ooops, no BNO055-2 detected ... Check your wiring or I2C ADDR!");
    ESP.restart();
  }
  bno.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
  printEvent("BNO2 is initialized.");

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    printEvent("Error initializing ESP-NOW.");
    ESP.restart();
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    printEvent("Failed to add peer.");
    ESP.restart();
  }
  printEvent("ESP-NOW setup finished.");

  // Set stepper motor speeds/accelerations
  LeadScrewStepper.setMaxSpeed(400);
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(400);
  CameraStepper.setMaxSpeed(200);
  CameraStepper.setAcceleration(1000);
  CameraStepper.setSpeed(200);

  printEvent("Setup done!");

  //Setup State, 0 = Failed, 1 = Success
  writeTrue();

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

  //Launched State, 0 = Failed, 1 = Success
  writeTrue();
  
  printEvent("Standing By for landing detection.");
  // Wait a minimum of 60 seconds before standing by for landing. Record flight data during this.
  for (int i = 1; i <= 10 * 90; i++) {
    if (i % 100 == 0){
      char timeMessage[50];
      snprintf(timeMessage, 50, "We are %d seconds into flight!", i/10);
      printEvent(timeMessage);
    }
    updateLanding();
    delay(100);
  }

  // Wait in standby mode and loop until landed
  printEvent("Standing By for Landing");

  //Seconds Passed, Waiting for Landing , 0 = Failed, 1 = Success
  writeTrue();

  updateLanding();
  for(int i = 0; i < 10 * 60 * 20; i++){
    if(checkLanding()) break;
    delay(100);
    updateLanding();  
  }
  printEvent("We Have Landed!");

  //Landing Detection , 0 = Failed, 1 = Success
  writeTrue();
  delay(1000);

  // After landing, check to make sure the payload tube is not rolling
  printEvent("Standing by for checkRoll.");
  checkRoll();
  printEvent("Check Roll Finished.");

  // System has agreed on check roll, 0 = Failed, 1 = Success
  writeTrue();

  imu::Quaternion q = bno.getQuat();
  float yy = q.y() * q.y();
  float roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + yy));
  float initialXAngle = 57.2958 * roll;

  char buffer[100];
  int ret = snprintf(buffer, sizeof buffer, "Landed at %f degrees. Horizontal motion started.", initialXAngle);
  printEvent(buffer);
  delay(500);

  // What degree did system land?, 0 = Failed to commit
  EEPROM.writeFloat(address, initialXAngle);
  Serial.print("Address at angle "); 
  Serial.println(address);                                                                                              
  address += sizeof(initialXAngle);
  Serial.println(address);                                                                                               
  EEPROM.commit();
  writeMillis();

  leadScrewRun();
  printEvent("Done deploying horizontally.");

  // Lead Screw Deployed, 0 = Failed, 1 = Success
  writeTrue();

  delay(2000);

  // Camera Deployed, 0 = Failed, 1 = Success
  writeTrue();

  printEvent("Standing By for Camera commands...");
	serialMessage = "";
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
    printEvent("Executing Radio Commands");
    interpretRadioString(serialMessage);
    printEvent("Done with all radio commands.");

    writeMillis();

    int addressIndex = address;
    for (int i = 0; i < 30; i++) {
      EEPROM.write(addressIndex, commands[i] >> 8);
      EEPROM.write(addressIndex + 1, commands[i] & 0xFF);
      Serial.print("Written to address "); Serial.println(addressIndex);
      addressIndex += 2;
    }

    // Interpreted Radio Commands, 0 = Failed, 1 = Success
    Serial.println(addressIndex);
    EEPROM.writeBool(addressIndex, true);
    addressIndex += sizeof(bool);                                                                                                                                                                                                                                                                                                                                                                                                                                         
    EEPROM.commit();

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
  return (accelAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE);
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
    else if(serialMessage == "reset storage") {
      for (int i = 0 ; i < EEPROM.length() ; i++) {
  	    EEPROM.write(i, 0);
        EEPROM.commit();
	     }
      Serial.println("Flashed EEPROM to 0!");
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

    // Within .5 radians of the two BNO055 roll values
    if (roll2 - .5 < roll && roll < roll2 + .5) {
      printEvent("Both IMUs have same data (within 0.5 radians).");
       
      // Both IMUs Agree, 0 = Failed, 1 = Success
      EEPROM.writeBool(address, true);
      address += sizeof(bool);                                                                                                                                                                                                                    
      EEPROM.commit();

      EEPROM.writeULong(address, millis());
      Serial.println(address);
      address += sizeof(unsigned long);                                                                                                       
      EEPROM.commit(); 

      // Store Final Roll 1
      EEPROM.writeFloat(address, roll);
      address += sizeof(roll);
      EEPROM.commit();
      // Store Final Roll 2
      EEPROM.writeFloat(address, roll2);
      address += sizeof(roll2);                                                                                                         
      EEPROM.commit();

      // Within 10 degrees of the two roll values on the first BNO055
      if (prevRoll - 10 < currentRoll && currentRoll < prevRoll + 10) {
        printEvent("prevRoll is within 10 degrees of currentRoll.");

      // IMU Agrees that prevRoll is same as last currentRoll, 0 = Failed, 1 = Success
      EEPROM.writeBool(address, true);
      address += sizeof(bool);                                                                                                                                                                                                                    
      EEPROM.commit();
      
      EEPROM.writeULong(address, millis());
      Serial.println(address);
      address += sizeof(unsigned long);                                                                                                       
      EEPROM.commit();

      // Store prevRoll
      EEPROM.writeFloat(address, prevRoll);
      address += sizeof(prevRoll);                                                                                                         
      EEPROM.commit();
      // Store currentRoll
      EEPROM.writeFloat(address, currentRoll);  
      address += sizeof(currentRoll);                                                                                                                                                                                                                 
      EEPROM.commit();
      // Store countCheck
      // EEPROM.writeInt(address, countCheck);
      // address += sizeof(countCheck);                                                                                                                                                                                                                 
      // EEPROM.commit();

        return true;
      }
    }
    else if (countCheck == 5) { // If a value can not come to a consesus within 5 polls, override the auxillary mechanism
      printEvent("Both IMU have different data.");
      if (prevRoll - 10 < currentRoll && currentRoll < prevRoll + 10) {
        printEvent("prevRoll is within 10 degrees of currentRoll.");

        // If all systems are working as intended, following values should be 0.
        // IMUs fail to agree, 0 = Failed, 1 = Success
        EEPROM.writeBool(address, true);
        address += sizeof(bool);                                                                                                                                                                                                                    
        EEPROM.commit();

        EEPROM.writeULong(address, millis());
        Serial.println(address);
        address += sizeof(unsigned long);                                                                                                       
        EEPROM.commit(); 

        // Store prevRoll
        EEPROM.writeFloat(address, roll);
        address += sizeof(roll);                                                                                                         
        EEPROM.commit();
        // Store currentRoll
        EEPROM.writeFloat(address, roll2);
        address += sizeof(roll2);                                                                                                                                                                                                                
        EEPROM.commit();

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
  int steps = angle / 1.8;
  cameraAngle += steps * 1.8;
  CameraStepper.move(steps);
  while (CameraStepper.run()) {}
}

void printEvent(const char *event) {
  Serial.print(millis());
  Serial.print(" - ");
  Serial.println(event);
}

void interpretRadioString(String message) { // "XX4XXX C3 A1 D4 C3 F6 C3 F6 B2 B2 C3."
  printEvent("Interpreting radio string");
  message.toUpperCase();
  int numberCommands = 0;
  while(1){
    int location = findFirstRadioCommand(message);
    if(location == -1) break;
    commands[numberCommands] = message.substring(location+1, location+2).toInt();
    numberCommands++;
    message.remove(location, 2);
  }

  if(numberCommands == 0)
    printEvent("No commands found in serial message.");
  for (int i = 0; i < numberCommands; i++) {
    executeRadioCommand(commands[i]);
    delay(3000);
  }
  sendData(0);
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
  char string[100];
  snprintf(string, 100, "I am executing command %d!", command);
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
  char timeString[20];
  snprintf(timeString, 20, "%lu", millis());


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


void writeTrue(){
  EEPROM.writeBool(address, true);
  address += sizeof(bool);                                                                                                                                                                                                                    
  EEPROM.commit();
  writeMillis();
}

void writeMillis() {
  EEPROM.writeULong(address, millis());
  address += sizeof(unsigned long);                                                                                                       
  EEPROM.commit();
}
