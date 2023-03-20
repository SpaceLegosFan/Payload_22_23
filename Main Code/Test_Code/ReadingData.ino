#include <EEPROM.h>

void readIntArrayFromEEPROM(int address, int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
    addressIndex += 2;
  }
}

void setup() {

  Serial.begin(38400);

  // Setup Check
  int setupState = EEPROM.read(0);
  Serial.print("Setup State: "); Serial.println(setupState);

  // Launch Check
  int launchState = EEPROM.read(1);
  Serial.print("Launch State: "); Serial.println(launchState);

  // Waiting for Landing
  int waitLandState = EEPROM.read(2);
  Serial.print("Waiting for Landing State: "); Serial.println(waitLandState);

  // Landing Check
  int landingState = EEPROM.read(3);
  Serial.print("Landing State: "); Serial.println(landingState);

  //



  const int ARRAY_SIZE = 5;
  const int STARTING_EEPROM_ADDRESS = 17;

  int newNumbers[ARRAY_SIZE];
  readIntArrayFromEEPROM(STARTING_EEPROM_ADDRESS, newNumbers, ARRAY_SIZE);

  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    Serial.println(newNumbers[i]);
  }
}

void loop() {}
