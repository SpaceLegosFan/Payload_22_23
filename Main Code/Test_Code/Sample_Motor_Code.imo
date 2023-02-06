// Running Motor Code
// last modified 10/28

/* Preliminary code to start to determine how to create planar motion with stepper motor
Based off example code from https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/ 
Other helpful websites: https://hackaday.io/project/183713/instructions
https://hackaday.io/project/183279-accelstepper-the-missing-manual */

//Link to wiring instructions: https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/ 

// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin_x 5
#define stepPin_x 2
#define enable_pin_x 8 // not sure what this is

#define dirPin_y 6 // need to figure out what these should be based on driver pins
#define stepPin_y 3
#define enable_pin_y 9

#define motorInterfaceType 1
 
// Create a new instance of the AccelStepper class:
AccelStepper stepper_x = AccelStepper(motorInterfaceType, stepPin_x, dirPin_x);
AccelStepper stepper_y = AccelStepper(motorInterfaceType, stepPin_y, dirPin_y);

// Define parameters for travel that will be based on input
// Likely need to be ints but we can increase precision by using microsteps
float positions_x[] = {0.0, 1.0, 0.5, 1, 0.5}; //arbitrary array of x positions
float positions_y[] = {0.0, 1.0, 0.5, 1, 0.5}; //arbitrary array of y positions

int oldPos_steps_x; //this will define previous x position in number of steps
float oldPos_inches_x; //this will define the previous x position in inches
float oldPos_mm_x; //this will define the previous x position in mm

int newPos_steps_x; // this will define next desired x position in number of steps
float newPos_inches_x; //this will define the new desired x posiiton in inches
float newPos_mm_x; //this will define the new desired x posiiton in mm

int travel_x; // this will define x amount that needs to be traveled (number of steps)

int oldPos_steps_y; //this will define previous y position in number of steps
float oldPos_inches_y; //this will define the previous y position in inches
float oldPos_mm_y; //this will define the previous y position in mm

int newPos_steps_y; // this will define next desired y position in number of steps
float newPos_inches_y; //this will define the new desired y posiiton in inches
float newPos_mm_y; //this will define the new desired y posiiton in mm

int travel_y; // this will define y amount that needs to be traveled (number of steps)


float in_to_mm = 25.400; 
int num_steps = 400; // this is set to 400 because we are currently half-stepping with hardware setup
int lead_mm = 5;
float steps_per_mm = float(num_steps) / float(lead_mm); // steps to travel 1 mm

bool go = true;

void setup() {
Serial.begin(9600);
  pinMode(enable_pin_x, OUTPUT);
  stepper_x.setEnablePin(enable_pin_x);
  stepper_x.setPinsInverted(false, false, true);  
  stepper_x.enableOutputs();

  pinMode(enable_pin_y, OUTPUT);
  stepper_y.setEnablePin(enable_pin_y);
  stepper_y.setPinsInverted(false, false, true);  
  stepper_y.enableOutputs();
  
  // Set the maximum speed in steps per second:
  stepper_x.setMaxSpeed(1000); //this is the equivalent of 5 revolutions/second, will have to determine what ideal value is
  stepper_y.setMaxSpeed(1000);

  oldPos_inches_x = positions_x[0]; // initial x position in inches
  oldPos_mm_x = oldPos_inches_x * in_to_mm; // initial x position in mm
  oldPos_steps_x = int(oldPos_mm_x * steps_per_mm); // set initial x position to zero. We will have robot start at origin

  oldPos_inches_y = positions_y[0]; // initial y position in inches
  oldPos_mm_y = oldPos_inches_y * in_to_mm; // initial y position in mm
  oldPos_steps_y = int(oldPos_mm_y * steps_per_mm); // set initial y position to zero. We will have robot start at origin

}

void loop() { 
  if (!go) return;

  // Note that one rotation of the motor will move the screw 5 mm

   for (int i = 1; i <= 4; i++) // start at i=1 because 2nd element is the first new position we need, this is just for testing
  {
  // Set the current position to 0
  stepper_x.setCurrentPosition(0); // this will 'normalize' each of the required movements
  stepper_y.setCurrentPosition(0);

  // Will somehow need to set this to run until data point reached
  // Also need to include conversion of inches to mm
  // Note that 5 mm = 0.19685 inches; 1 mm = 0.0393701 inches; 1 in = 25.4 mm
  // Right now, 200 steps per revolution; can change by implementing microsteps for more accuracy


  // add code here to read in point from server side
  // convert inches into mm, then into steps
  newPos_inches_x = positions_x[i]; //new x position is next element in array (this will be a read statement later down the road)
  newPos_mm_x = newPos_inches_x * in_to_mm;
  newPos_steps_x = int(steps_per_mm * newPos_mm_x); // this is how many steps the final x position needs -- have to figure out float and int stuff still
  travel_x = newPos_steps_x - oldPos_steps_x; // amount needs to move in x, will be negative if needs to move backwards, positive if needs to move forward

  newPos_inches_y = positions_y[i]; //new y position is next element in array (this will be a read statement later down the road)
  newPos_mm_y = newPos_inches_y * in_to_mm;
  newPos_steps_y = int(steps_per_mm * newPos_mm_y); // this is how many steps the final y position needs -- have to figure out float and int stuff still
  travel_y = newPos_steps_y - oldPos_steps_y; // amount needs to move in y, will be negative if needs to move backwards, positive if needs to move forward
  
  
  while((stepper_x.currentPosition() != travel_x) || (stepper_y.currentPosition() != travel_y)) //I think we want or here, not entirely sure
  {
    if ((travel_x < 0) && (travel_y < 0))
  {
    stepper_x.setSpeed(-200); // this will run at constant speed. Likely want to do this but we can decide
    stepper_y.setSpeed(-200);
    stepper_x.runSpeed();
    stepper_y.runSpeed();
  }
  else if ((travel_x >= 0) && (travel_y >= 0))
  {
    stepper_x.setSpeed(200); // this will run at constant speed. Likely want to do this but we can decide
    stepper_y.setSpeed(200);
    stepper_x.runSpeed();
    stepper_y.runSpeed();
  }
  else if ((travel_x <0) && (travel_y >=0))
  {
    stepper_x.setSpeed(-200);
    stepper_y.setSpeed(200);
    stepper_x.runSpeed();
    stepper_y.runSpeed();
  }
  else
  {
    stepper_x.setSpeed(200);
    stepper_y.setSpeed(-200);
    stepper_x.runSpeed();
    stepper_y.runSpeed();
  }
    
  }

  // Set what was newDataPoint in this iteration to now be the old/current datapoint
  oldPos_steps_x = newPos_steps_x;
  oldPos_steps_y = newPos_steps_y;

  delay (1000); // this delay statement is just to see motor waiting between points
  }
  go = false;
}
