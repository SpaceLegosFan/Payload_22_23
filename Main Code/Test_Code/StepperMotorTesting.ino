// NDRT Deployment Testing
// Link to example: https://microcontrollerslab.com/stepper-motor-a4988-driver-module-esp32/

#include <AccelStepper.h>

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

int movementDirection; // 1 for moving up |||| -1 for moving down
const int DIR = 2;
const int STEP = 3;

#define motorInterfaceType 1
AccelStepper LeadScrewStepper(motorInterfaceType, STEP, DIR);
//AccelStepper CameraStepper(motorInterfaceType, STEP, DIR);

// System Properties
float travel_distance = 8.63; //9.6;//8.63; // ask spencer or https://drive.google.com/drive/u/0/folders/1Yd59MVs0kGjNgtfuYpVg5CDFZwnHGlRj

// Lead Screw Properties 
// Part link https://www.mcmaster.com/8677N21/
float num_steps = 400; // steps per rotation; this would be if we are half-stepping (units: steps/revolution)
float travel_distance_per_full_step = 0.00125; // inches/step

//Motion Calculations
float num_deployment_LeadScrew_steps = movementDirection*travel_distance / travel_distance_per_full_step;

void setup() {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.begin(115200);

  Serial.begin(115200);
  Serial.println("Enter 1 to deploy lead screw cover or -1 to retract lead screw cover");
while (Serial.available() == 0) {
;
}
movementDirection = Serial.parseInt();
Serial.println(movementDirection);
delay(1000);

  // Lead Screw Stepper (Primary)
  LeadScrewStepper.setMaxSpeed(400); //800
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(500);
  LeadScrewStepper.moveTo(num_deployment_LeadScrew_steps);
  //LeadScrewStepper.runSpeedToPosition(); // Blocks until all are in position
}

void loop() {
// Change direction once the motor reaches target position
  //if (LeadScrewStepper.distanceToGo() == 0) 
    //LeadScrewStepper.moveTo(-LeadScrewStepper.currentPosition());

  // Move the motor one step
  LeadScrewStepper.run();
}
