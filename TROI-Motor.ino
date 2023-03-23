#include <AccelStepper.h>
// Need to change the pins for Arduino
#define motorInterfaceType 1
#define leadDIR 12 // DIR1 = 26
#define leadSTEP 14 // STEP1 = 25
#define cameraDIR 26  // DIR2 = 13
#define cameraSTEP 25 // STEP2 = 27
#define DEPLOYSTEPS 5700

// Motor Values
float travel_distance = 7.5;
float travel_distance_per_full_step = 0.00125;
float num_deployment_LeadScrew_steps = DEPLOYSTEPS;

AccelStepper LeadScrewStepper(motorInterfaceType, leadSTEP, leadDIR);
AccelStepper CameraStepper(motorInterfaceType, cameraSTEP, cameraDIR);
float cameraAngle = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  LeadScrewStepper.setMaxSpeed(400); // 800
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(400);
  CameraStepper.setMaxSpeed(200);
  CameraStepper.setAcceleration(1000);
  CameraStepper.setSpeed(200);

}

String serialMessage = "";

void loop(){
  updateSerialMessage();
  if(serialMessage != ""){
    checkSerialMessage();
  }
  delay(2000);
  serialMessage = "";
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
    else if (serialMessage.indexOf("steps =") != -1){
      num_deployment_LeadScrew_steps = serialMessage.substring(serialMessage.indexOf("=") + 2).toInt();
    }
    else if (serialMessage.indexOf("camera turn") != -1) {
      int angle = serialMessage.substring(serialMessage.indexOf("=") + 2).toInt();
      spinCameraStepper(angle);
    }
    else
      Serial.println("Not a valid command!");
    serialMessage = "";
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
