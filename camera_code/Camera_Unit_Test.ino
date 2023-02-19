/*****

  Jacob Elwell
  Camera Unit Test

  ***NOTE***

  This code will take 6 pictures consecutively upon reset, and will take them
  over the course of ~20 seconds. It can only store those 6 pictures and each time
  the ESP32-CAM is reset it will overwrite the previous .jpg files on the microSD.

  Make sure the microSD card is NOT inserted during code upload, but it must be
  inserted to run the code and save the pictures.

  For some reason, the camera settings are delayed in the file that they are written to, for example:

  Expected:       Actual:
  Normal 1        Normal
  Grayscale       Normal
  Flipped         Grayscale
  Negative        Flipped
  Normal 2        Negative
  Normal 3        Normal (included to help debug)
  
*****/



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


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);

  // Initialize the camera  
  Serial.print("Initializing the camera module...");
  DefCamSettings(config, s);
  Serial.println("Ok!");
 
  // Initialize MicroSD
  Serial.print("Initializing the MicroSD card module... ");
  initMicroSDCard();

  // Take three junk pictures, for some reason this affects the image quality
  String initial_path_junk = "/null.jpg";
  takeSavePhoto(initial_path_junk);
  Serial.println("Ok!");
  
  takeSavePhoto(initial_path_junk);
  
  takeSavePhoto(initial_path_junk);

  // Delete initialization photos
  deleteFile(SD_MMC, "/null.jpg");

  // Normal picture
  String pic1 = "/normal1.jpg";
  takeSavePhoto(pic1);
  Serial.println("Ok!");

  // Grayscale
  String pic2 = "/grayscale.jpg";
  color_2_gray(s);
  takeSavePhoto(pic2);
  Serial.println("Ok!");

  // Normal color, flipped 180 deg vertically
  String pic3 = "/flipped.jpg";
  gray_2_color(s);
  rotate_180(s, flip_set);
  takeSavePhoto(pic3);
  Serial.println("Ok!");

  // Special filter (negative image)
  String pic4 = "/negative.jpg";
  rotate_180(s, flip_set);
  spec_filt(s);
  takeSavePhoto(pic4);
  Serial.println("Ok!");

  // Remove all filters, normal picture again
  String pic5 = "/normal2.jpg";
  remove_filt(s);
  takeSavePhoto(pic5);
  Serial.println("Ok!");  

  String pic6 = "/normal3.jpg";
  takeSavePhoto(pic6);
}

void loop() {

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
  Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
}

void takeSavePhoto(String path){
  // Take Picture with Camera

  vTaskDelay(stut);
  
  camera_fb_t  * fb = esp_camera_fb_get();

  
  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Save picture to microSD card
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  
  //return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);
  
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void color_2_gray(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 2);

}

void gray_2_color(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  Serial.println("color complete");

}

void rotate_180(sensor_t * s, bool& flip_set){

  s = esp_camera_sensor_get();
  s->set_vflip(s, !flip_set);

  flip_set = !flip_set;

  Serial.println(flip_set);

}

void spec_filt(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 1);

  Serial.println("filter on");

}

void remove_filt(sensor_t * s){

  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  s->set_vflip(s, 0);

  Serial.println("all filters off");
  
}