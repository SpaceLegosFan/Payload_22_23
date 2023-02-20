#include "Arduino.h"
#include "TROI-CAM.h"

camclass::camclass(){
}

void camclass::take_picture(){

  Serial.println("Take picture");
  Serial.println();

}

void camclass::color_2_gray(){

  Serial.println("Color to grayscale");
  Serial.println();

}

void camclass::gray_2_color(){

  Serial.println("Grayscale to color");
  Serial.println();

}

void camclass::rotate_180(){

  Serial.println("Rotate image 180");
  Serial.println();

}

void camclass::spec_filt(){

  Serial.println("Special effects filter");
  Serial.println();

}

void camclass::remove_filt(){

  Serial.println("Remove all filters");
  Serial.println();

}

camclass espcam = camclass();
