#include "RTClib.h"
#define I2C_SDA 21
#define I2C_SCL 22

RTC_DS3231 rtc;
TwoWire I2CSensors = TwoWire(0);

void setup() {
  Serial.begin(115200);
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  Serial.println("I2CSensor initialized!");

  if (! rtc.begin(&I2CSensors)) {
    Serial.println("Couldn't find RTC");
    return;
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime now = rtc.now();
  now = now + TimeSpan(0, 0, 0, 15);
  delay(500);
  rtc.adjust(now);

  Serial.println("Date and Time Set!");
}

void loop () {
  DateTime now = rtc.now();
  char bufferString[] = "DD MMM hh:mm:ss";
  char *timeString = now.toString(bufferString);
  Serial.println(timeString);
}
