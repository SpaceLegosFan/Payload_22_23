/*****

  Jacob Elwell
  ESP32-Main wireless receive baseline.

  ***NOTE***

  Transmits two data points (timestamp and command #).


*****/

#include <esp_now.h>
#include <WiFi.h>

// This is the RECEIVER (ESP32-CAM) MAC address--change this!
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x9A, 0x83, 0x68};

// Structure example to receive data
// Must match the receiver structure

typedef struct struct_message {

    char timestamp[32];
    int command;

} struct_message;

// Create a struct_message called myData
struct_message myData;

// Receiver data structure
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup(){
  Serial.begin(115200);

  //  Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
}

void loop() {

  // Set values to send
  strcpy(myData.timestamp, "TIMESTAMP");

  // Dummy test; sends random "commands"
  myData.command = random(3, 9);

  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }

  else {
    Serial.println("Error sending the data");
  }

  // Sends new command every 2 seconds
  delay(2000);

}
