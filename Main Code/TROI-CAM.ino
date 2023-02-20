/*****

  Jacob Elwell
  ESP32-CAM wireless receive baseline.

  ***NOTE***

  Receives two data points (timestamp and command #) and passes those to a switch statement.
  Edit commands in the corresponding ESP32-CAM.cpp file
  

*****/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>


// This is the TRANSMITTER (ESP32-Main) MAC address--change this!
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0x31, 0x03, 0x34};

//  Structure example to send data
//  Must match the sender structure

typedef struct struct_message {

    char timestamp[32];
    int command;

} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  Serial.println(myData.command);

  // Switch decides which function to execute
  switch(myData.command){

    case 3:
      take_picture();
      break;

    case 4:
      color_2_gray();
      break;

    case 5:
      gray_2_color();
      break;

    case 6:
      rotate_180();
      break;

    case 7:
      spec_filt();
      break;

    case 8:
      remove_filt();
      break;

  }

}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initalize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}

void take_picture(){

  Serial.println("Take picture");
  Serial.println();

}

void color_2_gray(){

  Serial.println("Color to grayscale");
  Serial.println();

}

void gray_2_color(){

  Serial.println("Grayscale to color");
  Serial.println();

}

void rotate_180(){

  Serial.println("Rotate image 180");
  Serial.println();

}

void spec_filt(){

  Serial.println("Special effects filter");
  Serial.println();

}

void remove_filt(){

  Serial.println("Remove all filters");
  Serial.println();

}

