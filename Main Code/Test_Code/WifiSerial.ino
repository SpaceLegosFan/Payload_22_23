
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>

#define LED 2

AsyncWebServer server(80);

const char* ssid = "ND-guest";          // Your WiFi SSID
const char* password = "";  // Your WiFi Password

void recvMsg(uint8_t *data, size_t len){
  WebSerialPro.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerialPro.println(d);
  if (d == "ON"){
    digitalWrite(LED, HIGH);
  }
  if (d=="OFF"){
    digitalWrite(LED, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
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
}

void loop() {
  WebSerialPro.println("Hello!");
  delay(2000);
}
