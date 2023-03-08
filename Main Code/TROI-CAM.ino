// You need to install ESP32 1.0.6, not the current 2.0.x version. You also need to manually install the LittleFS library.

#include "esp_camera.h"
#include <Arduino.h>
#include <esp_now.h>
#include "img_converters.h"                                 // Txt overlay testing
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include <WiFi.h>

#include "SD_MMC.h"                                         // SD Card ESP32
#include "soc/soc.h"                                        // Disable brownout problems
#include "soc/rtc_cntl_reg.h"                               // Disable brownout problems
#include "driver/rtc_io.h"
#include <EEPROM.h>                                         // read and write from flash memory
#include <SPIFFS.h>
#include "FS.h"
#include "LITTLEFS.h"



// define the number of bytes you want to access
#define EEPROM_SIZE 512

// Pin definition for CAMERA_MODEL_AI_THINKER
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


#define FILE_PHOTO_FS "/data/txtOvl_photo.jpg"   

boolean takeNewPhoto = true;
bool taskCompleted = false;

int countReceived = 0;

String fileName = "empty";    


// Stores the camera configuration parameters
camera_config_t config;

// Stores camera sensor settings
sensor_t *s;

// Used for the flip 180 deg command
bool flip_set = 0;

//
int pictureNumber = 0;

String string = "";

// Time before the picture is taken each time the "take picture" command is called
const TickType_t stut = 3000 / portTICK_PERIOD_MS; // Change this to change the time, the whole number corresponds with milliseconds

// This is the TRANSMITTER (ESP32-Main) MAC address--change this!
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x69, 0xD0};

//  Structure example to send data
//  Must match the sender structure
typedef struct struct_message {
    char timestamp[32];
    int command;
} struct_message;

// Create a struct_message called myData
struct_message myData;

typedef unsigned char u8;

#define OV2640_MAXLEVEL_SHARPNESS 6

const static u8 OV2640_SHARPNESS_AUTO[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0x20, 0x20,
0x00, 0x00, 0x00
};

const static u8 OV2640_SHARPNESS_MANUAL[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0x00, 0x20,
0x00, 0x00, 0x00
};

const static u8 OV2640_SHARPNESS_LEVEL0[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc0, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL1[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc1, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL2[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc2, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL3[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc4, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL4[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xc8, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL5[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xd0, 0x1f,
0x00, 0x00, 0x00
};
const static u8 OV2640_SHARPNESS_LEVEL6[]=
{
0xFF, 0x00, 0xff,
0x92, 0x01, 0xff,
0x93, 0xdf, 0x1f,
0x00, 0x00, 0x00
};

const static u8 *OV_SETTING_SHARPNESS[]=
{
OV2640_SHARPNESS_LEVEL0,
OV2640_SHARPNESS_LEVEL1,
OV2640_SHARPNESS_LEVEL2,
OV2640_SHARPNESS_LEVEL3,
OV2640_SHARPNESS_LEVEL4,
OV2640_SHARPNESS_LEVEL5,
OV2640_SHARPNESS_LEVEL6
};

static int table_mask_write(const u8* ptab)
{
u8 address;
u8 value,orgval;
u8 mask;
const u8 *pdata=ptab;

if ( NULL==pdata )   
    return -1;
sensor_t * s = esp_camera_sensor_get();

while(1)   
{   
    address =*pdata++;   
    value = *pdata++;   
    mask = *pdata++;           
    if ( (0==address) && (0==value) &&(0==mask) )   
    {   
        break;   
    }   
    
    s->set_reg(s,address,mask,value);
}   

return 0;   

}

int change_sharpness( int sharpness )
{
if ( sharpness > OV2640_MAXLEVEL_SHARPNESS)
{
return -1;
}

if( sharpness <0 )   
{   
    table_mask_write(OV2640_SHARPNESS_AUTO);       
}   
else   
{   
    table_mask_write(OV2640_SHARPNESS_MANUAL);     
    table_mask_write(OV_SETTING_SHARPNESS[sharpness]);   
}      

return 0;   

}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  countReceived++;

  if(countReceived == 1) {

    initCAMERA(config, s);
    delay(5000);

  }

  Serial.println(myData.command);

  switch(myData.command) {
    case 3:
      take_picture();
      break;
    case 4:
      color_2_gray(s);
      break;
    case 5:
      gray_2_color(s);
      break;
    case 6:
      rotate_180(s, flip_set);
      break;
    case 7:
      spec_filt(s);
      break;
    case 8:
      remove_filt(s);
      break;
  }

}

 
static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    //fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, fb.height-20, color, str);
}


void initFS(){

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting FS!");
      ESP.restart();
  }  else {
    Serial.println("File System mounted successfully!");
      delay(500);
  }
}
  
void initSDCARD(){

    if(!SD_MMC.begin()){
  // if(!SD_MMC.begin("/sdcard", true)){               // note: ("/sdcard", true) = 1 wire - see: https://www.reddit.com/r/esp32/comments/d71es9/a_breakdown_of_my_experience_trying_to_talk_to_an/
      Serial.println("SD Card mount failed!");
      return;
    } else {
      Serial.println("SD Card mount successfull!");
    }
  

    uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
      Serial.println("No SD Card attached!");
        return;
    } else {
      Serial.println("SD Card attached!");
    }

  }

  
void initEEPROM(){

  if (!EEPROM.begin(EEPROM_SIZE)){                                  // Initialize EEPROM with predefined size
      Serial.println("failed to initialise EEPROM...");
      ESP.restart();
    } else {
      Serial.println("Success to initialise EEPROM...");
      pictureNumber = 0;
      EEPROM.put(8,pictureNumber);                                                                                                            
      EEPROM.commit();
    }

}
  

  
void initCAMERA(camera_config_t config, sensor_t * s){

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
    
    if(psramFound()){                                           // https://github.com/espressif/esp-who/issues/83
      Serial.println("PSRAM found!");
      //config.frame_size = FRAMESIZE_UXGA; 						        // FRAMESIZE_ + QVGA  ( 320 x 240 )
      config.frame_size = FRAMESIZE_SVGA; 						            // FRAMESIZE_ + QVGA  ( 320 x 240 ) 
      config.jpeg_quality = 1;                                 //              CIF   ( 352 x 288)
      config.fb_count = 1;                                      //              VGA   ( 640 x 480 )
    } else {                                                    //              SVGA  ( 800 x 600 )
      Serial.println("PSRAM not found!");                                 //              XGA   ( 1024 x 768 )
      config.frame_size = FRAMESIZE_SVGA;                              //              SXGA  ( 1280 x 1024 )
      config.jpeg_quality = 12;                                 //              UXGA  ( 1600 x 1200 )
      config.fb_count = 1;
    }

    if(psramInit()){
      Serial.println("PSRAM initiated!");
    } else {
      Serial.println("PSRAM initiation failed!");
      ESP.restart();
    }


    int tt = 0;
    esp_err_t err;
    do{
      err = esp_camera_init(&config);
        if (err != ESP_OK) {
          Serial.printf("Camera initalization failed with error 0x%x\n", err);
          Serial.printf("Init trial %i\n", tt);        
          tt++;
        } else {
          Serial.println("Camera init successfull");
        }
    } while (err != ESP_OK && tt<=20);

    if(tt >= 20){
      delay(500);
      ESP.restart();
      }
      /*


    s = esp_camera_sensor_get();
    s->set_brightness(s, 2);     // -2 to 2
    s->set_contrast(s, 2);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
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
      */

    camera_fb_t * fb = NULL;
    s = esp_camera_sensor_get(); 
    int light=255;
    int day_switch_value=140;

  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  //  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
 // s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  //s->set_ae_level(s, 2);       // -2 to 2
  //s->set_aec_value(s, 1200);    // 0 to 1200
  s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)6);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 0);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable    
 
    
   s->set_reg(s,0xff,0xff,0x01);//banksel    

   //light = s->get_reg(s,0x2f,0xff);
   Serial.print("First light is ");
   Serial.println(light);
   Serial.print("Old 0x0 value is");   
   Serial.println(s->get_reg(s,0x0,0xff));

     //light=120+cur_pic*10;
     //light=0+cur_pic*5;

    if(light<day_switch_value)
    {  
      

      // Registry sets at light<210 automatically if light levels detected below 140.
      s->set_reg(s,0x2d,0xff,0x0);//extra lines
      s->set_reg(s,0x2e,0xff,0x0);//extra lines
      s->set_reg(s,0x47,0xff,0x0);//Frame Length Adjustment MSBs
      s->set_reg(s,0x46,0xff,0x98);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x60);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)

      /*
      //here we are in night mode
      if(light<45)s->set_reg(s,0x11,0xff,1);//frame rate (1 means longer exposure)
      s->set_reg(s,0x13,0xff,0);//manual everything
      s->set_reg(s,0x0c,0x6,0x8);//manual banding
           
      s->set_reg(s,0x45,0x3f,0x3f);//really long exposure (but it doesn't really work)
      */
    }
    else
    {
      //here we are in daylight mode
      
      s->set_reg(s,0x2d,0xff,0x0);//extra lines
      s->set_reg(s,0x2e,0xff,0x0);//extra lines

      s->set_reg(s,0x47,0xff,0x0);//Frame Length Adjustment MSBs

    if(light<150)
    {
      s->set_reg(s,0x46,0xff,0xd0);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0xff);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0xff);//exposure (doesn't seem to work)
    }
    else if(light<160)
    {
      s->set_reg(s,0x46,0xff,0xc0);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0xb0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    }    
    else if(light<170)
    {
      s->set_reg(s,0x46,0xff,0xb0);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    }    
    else if(light<180)
    {
      s->set_reg(s,0x46,0xff,0xa8);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<190)
    {
      s->set_reg(s,0x46,0xff,0xa6);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x90);//exposure (doesn't seem to work)
    } 
    else if(light<200)
    {
      s->set_reg(s,0x46,0xff,0xa4);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<210)
    {
      s->set_reg(s,0x46,0xff,0x98);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x60);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<220)
    {
      s->set_reg(s,0x46,0xff,0x80);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x20);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<230)
    {
      s->set_reg(s,0x46,0xff,0x70);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x20);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<240)
    {
      s->set_reg(s,0x46,0xff,0x60);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x20);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x80);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    } 
    else if(light<253)
    {
      s->set_reg(s,0x46,0xff,0x10);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x40);//line adjust
      s->set_reg(s,0x45,0xff,0x10);//exposure (doesn't seem to work)
    }
    else
    {
      s->set_reg(s,0x46,0xff,0x0);//Frame Length Adjustment LSBs
      s->set_reg(s,0x2a,0xff,0x0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x0);//line adjust
      s->set_reg(s,0x45,0xff,0x0);//exposure (doesn't seem to work)
      s->set_reg(s,0x10,0xff,0x0);//exposure (doesn't seem to work)
    }
                                        
    s->set_reg(s,0x0f,0xff,0x4b);//no idea
    s->set_reg(s,0x03,0xff,0xcf);//no idea
    s->set_reg(s,0x3d,0xff,0x34);//changes the exposure somehow, has to do with frame rate

    s->set_reg(s,0x11,0xff,0x0);//frame rate
    s->set_reg(s,0x43,0xff,0x11);//11 is the default value     
    }
    
   Serial.println("Getting first frame at");
   Serial.println(millis());    
  fb = esp_camera_fb_get();
    //skip_frame();
   Serial.println("Got first frame at");
   Serial.println(millis());    

    if(light==0)
    {
      s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xf0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
    }
    else if(light==1)
    {
      s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xd0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
    }
    else if(light==2)
    {
      s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xb0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust  
    }
    else if(light==3)
    {
      s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust      
    }    
    else if(light==4)
    {
      s->set_reg(s,0x47,0xff,0x40);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust
    }
    else if(light==5)
    {
      s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light==6)
    {
      s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }         
    else if(light==7)
    {
      s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x30);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light==8)
    {
      s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x20);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }     
    else if(light==9)
    {
      s->set_reg(s,0x47,0xff,0x20);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x10);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }    
    else if(light==10)
    {
      s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<=12)
    {
      s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<=14)
    {
      s->set_reg(s,0x47,0xff,0x10);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }          
    else if(light<=18)
    {
      s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xb0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<=20)
    {
      s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }      
    else if(light<=23)
    {
      s->set_reg(s,0x47,0xff,0x08);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }           
    else if(light<=27)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xd0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<=31)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }     
    else if(light<=35)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x60);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }    
    else if(light<=40)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<45)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x40);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    //after this the frame rate is higher, so we need to compensate
    else if(light<50)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0xa0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }    
    else if(light<55)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x70);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<65)
    {
      s->set_reg(s,0x47,0xff,0x04);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x30);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }
    else if(light<75)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x80);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xff);//line adjust        
    }                 
    else if(light<90)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x50);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0xbf);//line adjust        
    }
    else if(light<100)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x20);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x8f);//line adjust        
    } 
    else if(light<110)
    {
      s->set_reg(s,0x47,0xff,0x02);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x10);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x7f);//line adjust        
    }      
    else if(light<120)
    {
      s->set_reg(s,0x47,0xff,0x01);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x10);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x5f);//line adjust        
    }     
    else if(light<130)
    {
      s->set_reg(s,0x47,0xff,0x00);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x2f);//line adjust        
    }
    else if(light<140)
    {
      s->set_reg(s,0x47,0xff,0x00);//Frame Length Adjustment MSBs
      s->set_reg(s,0x2a,0xf0,0x0);//line adjust MSB
      s->set_reg(s,0x2b,0xff,0x0);//line adjust        
    }
                   
    if(light<day_switch_value)s->set_reg(s,0x43,0xff,0x40);//magic value to give us the frame faster (bit 6 must be 1)

    //fb = esp_camera_fb_get();
   
    s->set_reg(s,0xff,0xff,0x00);//banksel 
    s->set_reg(s,0xd3,0xff,0x8);//clock

    //s->set_reg(s,0xff,0xff,0x00);//banksel
    //s->set_reg(s,0xd3,0xff,5);//clock
    
    s->set_reg(s,0x42,0xff,0x2f);//image quality (lower is bad)
    s->set_reg(s,0x44,0xff,3);//quality
    
    //s->set_reg(s,0x96,0xff,0x10);//bit 4, disable saturation


    //s->set_reg(s,0xbc,0xff,0xff);//red channel adjustment, 0-0xff (the higher the brighter)
    //s->set_reg(s,0xbd,0xff,0xff);//green channel adjustment, 0-0xff (the higher the brighter)
    //s->set_reg(s,0xbe,0xff,0xff);//blue channel adjustment, 0-0xff (the higher the brighter)
    
    //s->set_reg(s,0xbf,0xff,128);//if the last bit is not set, the image is dim. All other bits don't seem to do anything but ocasionally crash the camera

    //s->set_reg(s,0xa5,0xff,0);//contrast 0 is none, 0xff is very high. Not really useful over 20 or so at most.

    //s->set_reg(s,0x8e,0xff,0x30);//bits 5 and 4, if set make the image darker, not very useful
    //s->set_reg(s,0x91,0xff,0x67);//really weird stuff in the last 4 bits, can also crash the camera           

    //no sharpening
    //s->set_reg(s,0x92,0xff,0x1);
    //s->set_reg(s,0x93,0xff,0x0);  
    change_sharpness(0);

  
}




 
void takeImage(){

  vTaskDelay(stut);
  delay(1000);
  Serial.println("----------------------------");
  Serial.println("Taking a photo...");
     
  camera_fb_t * fb = esp_camera_fb_get();

  bool CamCaptureSucess = true;                                      
  bool CamCaptureSize = true;

  fb = esp_camera_fb_get();                                          
    if(!fb) {
      CamCaptureSucess = false;
        Serial.println("Camera capture failed...");

    } else {
      CamCaptureSucess = true;
        Serial.println("Camera capture success...");

        EEPROM.get(8,pictureNumber);                                            
        Serial.printf("Current picture number counter : %i\n",pictureNumber);
        
        pictureNumber++;                                                    
        EEPROM.put(8,pictureNumber);                                                                                                            
        EEPROM.commit();                                                                                                                                        
        EEPROM.get(8,pictureNumber);                                          
        Serial.printf("New picture number counter : %i\n",pictureNumber);

    } 

      // Text overlay
        String text(myData.timestamp);
        String txtOverlay = "TROI - " + text;
        const char* txtOverlay_char = txtOverlay.c_str();

        dl_matrix3du_t * image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item); // This throws the error which causes the panic at high levels of clock.
        rgb_print(image_matrix, 0x000000FF, txtOverlay_char);            

        size_t _jpg_buf_len = 0;
        uint8_t * _jpg_buf = NULL;
        fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
          //int available_PSRAM_size = ESP.getFreePsram();
          int available_PSRAM_size = ESP.getFreeHeap();
          Serial.printf("PSRAM Size available (bytes)           : %i\n", available_PSRAM_size);
          dl_matrix3du_free(image_matrix);

          int available_PSRAM_size_after = ESP.getFreeHeap();
          Serial.printf("PSRAM Size available after free (bytes): %i\n", available_PSRAM_size_after);
      // Text overlay END

  if (CamCaptureSucess == true){ 

    int t = 0;
    unsigned int pic_sz = 0;
      
    File fileFS = SPIFFS.open(FILE_PHOTO_FS, FILE_WRITE);             // Save picture on FS
    Serial.print(" ");
    Serial.printf("Picture file name (FS): %s\n", FILE_PHOTO_FS);               // FS Photo file name
    // Insert the data in the photo file
    if (!fileFS) {
      Serial.println("Failed to open file (FS) in writing mode!\n");
    } else {
      Serial.printf("File (FS) open in writing mode : %s\n",FILE_PHOTO_FS);

        do{
        fileFS.write(_jpg_buf, _jpg_buf_len);
        Serial.printf("The picture has been saved in (FS) ");
        pic_sz = fileFS.size();                                     
          Serial.printf("File Size: %i bytes | read trial: %i of 20\n", pic_sz, t);
          t++;
        } while (pic_sz == 0 && t <= 20);
          if(t >= 20){ESP.restart();}                                
    }

    fileFS.close();

   
    if ( pic_sz > 0 ){
      Serial.printf("Picture size is valid ... \n");
        CamCaptureSize = true;
    } else {
      Serial.printf("Picture size is not valid ...\n");
        CamCaptureSize = false;
    };

      
    if(CamCaptureSize == true){

        String path = "/picture" + String(pictureNumber) +".jpg";             // Path where new picture will be saved in SD Card 

        fs::FS &fs = SD_MMC;
        fileName = path.c_str();
          Serial.printf("Picture file name (SDCARD): %s\n", path.c_str());
        
        File fileSDCARD = fs.open(path.c_str(), FILE_WRITE);                  // Save image on SD Card with dynamic name
          if(!fileSDCARD){
            fileName = "fail to save pic";
              Serial.printf("Failed to open file (SDCARD) in writing mode...\n");
          } else {
            fileSDCARD.write(_jpg_buf, _jpg_buf_len); // payload (image), payload length
              Serial.printf("The picture has been saved in (SDCARD) : %s\n", path.c_str());
          }
          
          fileSDCARD.close();


        esp_camera_fb_return(fb);
    
    } //CamCaptureSize = true;
        
  } //CamCaptureSucess = false;

  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
 // pinMode(4, OUTPUT);                   // Those lines must be commented to allow
  //digitalWrite(4, LOW);                 //  the SD card pin to be released
  //rtc_gpio_hold_en(GPIO_NUM_4);         // If not commented, SD card file saving on
                                           //  2nd loop will result in critical fault

 delay(100);

}



 
void setup() {

  Serial.begin(115200);                                       // Start serial communication at 115200 baud 
  delay(5000); 
  WiFi.mode(WIFI_STA);



  // Initalize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    printEvent("Error initializing ESP-NOW.");
    ESP.restart();
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 				          //disable brownout detector

  initSDCARD();
  initFS();
  initCAMERA(config, s);
  initEEPROM();

  Serial.printf("Internal heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

  printEvent("Taking junk photos...");
  takeImage();
  delay(10000);
  takeImage();
  delay(10000);
  takeImage();

  strcpy(myData.timestamp, "Testing String");
}

void loop() {
  
}

void color_2_gray(sensor_t * s) {
  printEvent("Color to grayscale"); 
  s = esp_camera_sensor_get();
  s->set_special_effect(s, 2);
  printEvent("Ok!");
}

void gray_2_color(sensor_t * s) {
  gray_2_color(s);
  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  printEvent("Color Complete!");
  printEvent("Ok!");
}

void rotate_180(sensor_t * s, bool& flip_set) {
  printEvent("Rotate image 180");
  s = esp_camera_sensor_get();
  s->set_vflip(s, !flip_set);

  flip_set = !flip_set;

  Serial.println(flip_set);
  printEvent("Ok!");
}

void spec_filt(sensor_t * s) {
  printEvent("Special effects filter");
  s = esp_camera_sensor_get();
  s->set_special_effect(s, 1);
  printEvent("Filter on.");
  printEvent("Ok!");
}

void remove_filt(sensor_t * s) {
  printEvent("Remove all filters");
  s = esp_camera_sensor_get();
  s->set_special_effect(s, 0);
  s->set_vflip(s, 0);

  printEvent("All filters off.");
  printEvent("Ok!");
}

void take_picture() {
  printEvent("Take picture.");
  long int t1 = millis();
  takeImage();
  long int t2 = millis();
  Serial.print("Time taken to take picture: "); Serial.print(t2-t1); Serial.println(" milliseconds");
  printEvent("Ok!");
}


void printEvent(const char* event) {
  Serial.println(event);
}


//
