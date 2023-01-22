/*
  tcoverst

  Programmed on an Espressif ESP32-Wroom-32E and DRA818V

  Libary is a modified version of the Arduino-DRA818 Library 
*/

#include <stdio.h>
#include <SoftwareSerial.h>
#include "DRA818.h"

/* Used Pins */
#define DRApower      27    // to the DRA818 power pin
#define RX            16    // ESP32 U2UXD serial RX pin to the DRA818 TX pin
#define TX            17    // ESP32 U2UXD serial TX pin to the DRA818 RX pin
#define audioPin      33    // Audio value

SoftwareSerial *dra_serial; // Serial connection to DRA818
DRA818 *dra;                // the DRA object once instanciated
float freq = 148.000;       // the frequency to scan
float Rval;
float Tval;

void setup(){
  
  Serial.begin(9600); // for logging

  Serial.println("Booting ...");
  pinMode(DRApower, OUTPUT);                      // Power control of the DRA818
  digitalWrite(DRApower,HIGH);                    // start at high power
  Serial.print("initializing I/O ... ");  
  dra_serial = new SoftwareSerial(RX, TX);        // Instantiate the Software Serial Object, using ESP32 U2UXD pins
  
  Serial.println("done");

  Serial.print("initializing DRA818 ... ");
  /*
   * Configure DRA818V using 145.500 MHz, squelch 4, volume 8, no ctcss, 12.5 kHz bandwidth, all filters activated
   *
   * Alternative call:
   *  dra = new DRA818(dra_serial, DRA818_VHF);
   *  dra->handshake();
   *  dra->group(DRA818_12K5, 145.500, 145.500, 0, 4, 0);
   *  dra->volume(8);
   *  dra->filters(true, true, true);
   */
  dra = DRA818::configure(dra_serial, DRA818_VHF, freq, freq, 4, 8, 0, 0, DRA818_12K5, true, true, true);
  if (!dra) {
    Serial.println("\nError while configuring DRA818");
  }

  Serial.println("done");

  Serial.println("Starting ... ");
  Serial.print("Freq ");
  Serial.println(freq);
  char buf[9];

  if (!dra) return; // do nothing if DRA configuration failed

  dtostrf(freq, 8, 4, buf);  // convert frequency to string with right precision
  /* scan the frequency */
  if (dra->scan(freq)) Serial.println("Found");

  //DRA818V sometimes needs to be turnned on and off for the new code to register
  digitalWrite(DRApower,LOW);                    // set DRA to low power
  delay(100);
  digitalWrite(DRApower,HIGH);                   // set DRA to high power
}

void loop(){
  
  Rval = analogRead(audioPin);
  //float Rvoltage = Rval * (5.0 / 4095.0);  
  Serial.print("Freq: ");
  Serial.print(freq); 
  Serial.print(" Rx: ");
  Serial.print(Rval);
  // Visual debugging aid
  if (Rval > 1870){
    Serial.print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
  }
  Serial.println("");
  delay(50);
}
