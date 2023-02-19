// NDRT Deployment Testing
// Link to example: https://microcontrollerslab.com/stepper-motor-a4988-driver-module-esp32/

#include <AccelStepper.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>



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


AsyncWebServer server(80);

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


const char* ssid = "ND-guest";          // Your WiFi SSID - MUST BE CONNECTED TO ND-GUEST
const char* password = "";  // Your WiFi Password

void recvMsg(uint8_t *data, size_t len){
  WebSerialPro.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  if (d == "Up"){
    movementDirection = 1;
    WebSerialPro.println("Deploying Down!");
    Serial.println("Deploying Down!");

  }
  if (d=="Down"){
  movementDirection = -1;
  WebSerialPro.println("Deploying Up!");
  Serial.println("Deploying Up!");
  }
}


void setup() {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.begin(115200);

float num_deployment_LeadScrew_steps = movementDirection*travel_distance / travel_distance_per_full_step;

movementDirection = Serial.parseInt();
Serial.println(movementDirection);
delay(1000);

  // Lead Screw Stepper (Primary)
  LeadScrewStepper.setMaxSpeed(400); //800
  LeadScrewStepper.setAcceleration(1000);
  LeadScrewStepper.setSpeed(500);
  LeadScrewStepper.moveTo(num_deployment_LeadScrew_steps);
  //LeadScrewStepper.runSpeedToPosition(); // Blocks until all are in position

   WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerialPro.begin(&server);
  WebSerialPro.msgCallback(recvMsg);
  server.begin();

  delay(5000);

  Serial.println("Enter 1 to deploy lead screw cover or -1 to retract lead screw cover");
  WebSerialPro.println("Enter ""Up"" to deploy lead screw cover or ""Down"" to retract lead screw cover");

  while (Serial.available() == 0) { 
;
}

}

void loop() {
// Change direction once the motor reaches target position
  //if (LeadScrewStepper.distanceToGo() == 0) 
    //LeadScrewStepper.moveTo(-LeadScrewStepper.currentPosition());

  // Move the motor one step
  LeadScrewStepper.run();
}
