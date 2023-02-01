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

// Pins for full-scale lead screw(color of the heat shrinks)
//B2 YELLLOW
//A2 WHITE
//A1 BLUE
//B1 RED



#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// This code defines the DIR and STEP pin for the stepper motor (Dependent on the pins we solder) in order to initialize an instance of the AccelStepper class
const int DIR = 2;
const int STEP = 4;
const int steps_per_rev = 200;
const int MOTOR_LINEAR_SPEED = 1000; // Speed for when the payload is extending horizontally outside the payload tube
const int MOTOR_ROTATION_SPEED = 200; // Speed for when the payload is already fully extended outside the payload tube and is spinning to get to the correct orientation
#define motorInterfaceType 1
AccelStepper LeadScrewStepper(motorInterfaceType, STEP, DIR);
float travel_distance = 9.6;//8.63; // ask spencer or https://drive.google.com/drive/u/0/folders/1Yd59MVs0kGjNgtfuYpVg5CDFZwnHGlRj
// Lead Screw Properties 
// Part link https://www.mcmaster.com/8677N21/
float num_steps = 400; // steps per rotation; this would be if we are half-stepping (units: steps/revolution)
float travel_distance_per_full_step = 0.00125; // inches/step
//Motion Calculations
float num_deployment_LeadScrew_steps = travel_distance / travel_distance_per_full_step;

#define I2C_SDA 14
#define I2C_SCL 15
TwoWire I2CSensors = TwoWire(0);

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Create instance of BNO055 sensor



// Setup for Hunter's code

// We need to pull data from two accelerometers and aggregate it, perhaps a % comparison value?
imu::Vector<3>* accelerationQueue;
imu::Vector<3>* gyroQueue;
int size;
const float ACCELERATION_LAND_TOLERANCE = .3;
const float GYRO_LAND_TOLERANCE = 5;
const float ACCELERATION_LAUNCH_TOLERANCE = 20;
const int TIME_BETWEEN_UPDATES = 100; // time in ms
const float SPINNING_DEGREE_THRESHOLD = 5;

void setup()
{
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.begin(115200);
  LeadScrewStepper.setMaxSpeed(MOTOR_LINEAR_SPEED); // Sets the max speed the stepper motor can reach
  LeadScrewStepper.setAcceleration(100); // Sets the acceleration rate of the stepper motor
  LeadScrewStepper.setMaxSpeed(800);
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(500);
  LeadScrewStepper.moveTo(num_deployment_LeadScrew_steps);
  
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  

  // Ensures the system, accerometer, and gyroscope are calibrated adequately
  Serial.println("Orientation Sensor Testing...\n");
  uint8_t system, gyro, accel, mag = 0;
  while(/*system < 1 || */gyro < 1 || accel < 1){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print("\n");
    delay(200);
  }

  // Setup for Hunter's code
  size = 0;
  accelerationQueue = new imu::Vector<3>[10];
  gyroQueue = new imu::Vector<3>[10];


  
  Serial.print("Setup done\n");
  delay(1000);




  // wait in standby mode and loop until takeoff
  Serial.print("Standing By for Launch\n");
  bool standby = true;
  while (standby == true)
  {
    updateLaunch();
    standby = !checkLaunch();
    if (standby == false){
      delay(TIME_BETWEEN_UPDATES);
    }
  }
  Serial.print("We Have Launched!\n");




  // wait in standby mode and loop until landed
  Serial.print("Standing By for Landing\n");
  standby=true;
  while(standby == true){
    updateLanding();

    standby = !checkLanding;
    if(standby == false){
      delay(TIME_BETWEEN_UPDATES);
    }
  }
  Serial.print("We Have Landed!\n");
  delay(1000);




  // After landing, deploy horizontally
  Serial.print("Deploying Horizontlally. Standby for spinning.\n");
  standby = true;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  float yy = quat.y() * quat.y();
  float roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + yy));
  float prevXAngle = 57.2958 * roll;
  delay(100);
  LeadScrewStepper.run();
  while(standby == true){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  float yy1 = quat.y() * quat.y();
  float roll1 = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + yy1));
  float xAngle = 57.2958 * roll1;    
  float xAngleDifference = xAngle - prevXAngle;
    Serial.print(xAngleDifference);
    Serial.print("\n");
    if(xAngleDifference < 0){
      xAngleDifference = -xAngleDifference;
    }
    if (xAngleDifference > SPINNING_DEGREE_THRESHOLD) {
      Serial.println("Spinning detected!");
      standby=false;
    }
    prevXAngle = xAngle;
    delay(TIME_BETWEEN_UPDATES);
  }
  delay(1000);




  // Orient
  Serial.print("Orienting\n");
  float yy2 = quat.y() * quat.y();
  float roll2 = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + yy2));
  float eulerx = 57.2958 * roll2;    
  //LeadScrewStepper.setMaxSpeed(MOTOR_ROTATION_SPEED);
  //LeadScrewStepper.setSpeed(MOTOR_ROTATION_SPEED);
  //LeadScrewStepper.run(); // Spins (clockwise?) (To spin counterclockwise, set the motor speed above to -MOTOR_LINEAR_SPEED)
  while (!(eulerx > 269 && eulerx < 271))
  {
    //Serial.print(euler.x());
    Serial.print("\n");
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  }
  //LeadScrewStepper.stop();
  Serial.print("Orientation Complete\n");
  delay(1000);



  // deploy vertically
  Serial.print("Deploying Vertically\n");
  /*

    Code to deploy vertically goes here

  */
  delay(5000);
  Serial.print("Standing By for Camera commands...\n");
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
  if (accelerationAverage < ACCELERATION_LAND_TOLERANCE && gyroAverage < GYRO_LAND_TOLERANCE) {
    return true;
  } else {
    return false;
  }
}
