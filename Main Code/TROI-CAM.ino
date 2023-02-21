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
#include <WebSerialPro.h>
#include "esp_camera.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
// Change pin definition if you're using another ESP32 with camera module
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// WiFi
AsyncWebServer server(80);
const char* ssid = "\x48\x75\x6e\x74\x65\x72\xe2\x80\x99\x73\x20\x69\x50\x68\x6f\x6e\x65"; // Your WiFi SSID
const char* password = "hunter123";  // WiFi Password
const char* ssid_backup = "ND-guest";
const char* password_backup = "";

// Keep track of number of pictures
unsigned int pictureNumber = 0;

// Stores the camera configuration parameters
camera_config_t config;

// Stores camera sensor settings
sensor_t *s;

// Used for the flip 180 deg command
bool flip_set = 0;

// Time before the picture is taken each time the "take picture" command is called
const TickType_t stut = 3000 / portTICK_PERIOD_MS; // Change this to change the time, the whole number corresponds with milliseconds

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
  WebSerialPro.println(myData.command);

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

  // Initalize WiFi for WebSerialPro. Accessible at "<IP Address>/webserial" in browser
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

  // Initalize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    printEvent("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  // Initialize the camera  
  printEvent("Initializing the camera module...");
  DefCamSettings(config, s);
  printEvent("Ok!");
 
  // Initialize MicroSD
  printEvent("Initializing the MicroSD card module... ");
  initMicroSDCard();
  printEvent("Ok!");

  // Take three junk pictures, for some reason this affects the image quality
  printEvent("Taking junk photos...");
  String initial_path_junk = "/null.jpg";
  takeSavePhoto(initial_path_junk);
  takeSavePhoto(initial_path_junk);
  takeSavePhoto(initial_path_junk);

  // Delete initialization photos
  deleteFile(SD_MMC, "/null.jpg");
  printEvent("Ok!");
}
 
void loop() {

}

void recvMsg(uint8_t *data, size_t len) {
  WebSerialPro.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerialPro.println(d);
  d.toLowerCase();
  if(d.indexOf("camera command = ") != -1){
    int radioCommand = d.substring(d.indexOf("=") + 2).toInt();
    WebSerialPro.print("The radio command is: ");
    WebSerialPro.println(radioCommand);
    switch(radioCommand){
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
}

void take_picture() {
  printEvent("Take picture.");
  String pic1 = "/normal1.jpg";
  takeSavePhoto(pic1);
  printEvent("Ok!");
}

void color_2_gray() {
  printEvent("Color to grayscale");
  String pic2 = "/grayscale.jpg";
  color_2_gray(s);
  takeSavePhoto(pic2);
  printEvent("Ok!");
}

void gray_2_color() {
  printEvent("Grayscale to color");
  String pic7 = "/grayscaletocolor.jpg";
  gray_2_color(s);
  takeSavePhoto(pic7);
  printEvent("Ok!");
}

void rotate_180(){
  printEvent("Rotate image 180");
  String pic3 = "/flipped.jpg";
  gray_2_color(s);
  rotate_180(s, flip_set);
  takeSavePhoto(pic3);
  printEvent("Ok!");
}

void spec_filt(){
  printEvent("Special effects filter");
  String pic4 = "/negative.jpg";
  rotate_180(s, flip_set);
  spec_filt(s);
  takeSavePhoto(pic4);
  printEvent("Ok!");
}

void remove_filt(){
  printEvent("Remove all filters");
  String pic5 = "/normal2.jpg";
  remove_filt(s);
  takeSavePhoto(pic5);
  printEvent("Ok!");  
  String pic6 = "/normal3.jpg";
  takeSavePhoto(pic6);
}

void DefCamSettings(camera_config_t config, sensor_t * s){
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 12000000;
  config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG

  // Select lower framesize if the camera doesn't support PSRAM
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10; //10-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 60;
    config.fb_count = 1;
  }
  
  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    WebSerialPro.printf("Camera init failed with error 0x%x", err);
    return;
  }

  s = esp_camera_sensor_get();
  s->set_brightness(s, 2);     // -2 to 2
  s->set_contrast(s, 2);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 0);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
}

void initMicroSDCard(){
  // Start Micro SD card
  printEvent("Starting SD Card");
  if(!SD_MMC.begin()){
    printEvent("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    printEvent("No SD Card attached");
    return;
  }
}

void takeSavePhoto(String path){
  // Take Picture with Camera

  vTaskDelay(stut);
  
  camera_fb_t  * fb = esp_camera_fb_get();

  
  
  if(!fb) {
    printEvent("Camera capture failed.");
    return;
  }

  // Save picture to microSD card
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    printEvent("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    WebSerialPro.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  
  //return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);
  
}

void deleteFile(fs::FS &fs, const char * path){
      Serial.printf("Deleting file: %s\n", path);
      WebSerialPro.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
       printEvent("File deleted.");
    } else {
        printEvent("Delete failed.");
    }
}

void color_2_gray(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 2);

}

void gray_2_color(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  printEvent("Color Complete!");

}

void rotate_180(sensor_t * s, bool& flip_set){

  s = esp_camera_sensor_get();
  s->set_vflip(s, !flip_set);

  flip_set = !flip_set;

 Serial.println(flip_set);
 WebSerialPro.println(flip_set);

}

void spec_filt(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 1);

 printEvent("Filter on.");

}

void remove_filt(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  s->set_vflip(s, 0);

  printEvent("All filters off.");
  
}

void printEvent(const char* event){
  Serial.println(event);
  WebSerialPro.println(event);
}
