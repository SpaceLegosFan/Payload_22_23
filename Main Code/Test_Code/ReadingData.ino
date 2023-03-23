#include <EEPROM.h>

int address = 0;
int addressIndex;

void readIntArrayFromEEPROM(int addressing, int numbers[], int arraySize)
{
  addressIndex = addressing;
  for (int i = 0; i < arraySize; i++)
  {
    numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
    addressIndex += 2;
  }
}

void setup() {

  Serial.begin(38400);

  EEPROM.begin(512);

  // Setup Check
  Serial.print("Setup State: ");  Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // Launch Check
  Serial.print("Launch State: ");   Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // Waiting for Landing
  Serial.print("Waiting for Landing State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // Landing Check
  Serial.print("Landing State: "); Serial.println(EEPROM.readBool(address));
   address += sizeof(bool);

  // IMUs Agree
  Serial.print("IMU Agree State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);
  
  // Roll1 Value
  Serial.print("Roll1 Value: "); Serial.println(EEPROM.readFloat(address), 4);
  address += sizeof(float);

  // Roll2 Value
  Serial.print("Roll2 Value: "); Serial.println(EEPROM.readFloat(address), 4);
  address += sizeof(float);

  // Roll Agree State
  Serial.print("Roll Agree State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // prevRoll Value
  Serial.print("prevRoll Value: "); Serial.println(EEPROM.readFloat(address), 4);
  address += sizeof(float);

  // current Roll Value
  Serial.print("currentRoll Value: "); Serial.println(EEPROM.readFloat(address), 4);
  address += sizeof(float);

  // Count Check Value
  Serial.print("countCheck Value: "); Serial.println(EEPROM.readInt(address));
  address += sizeof(int);

  // checkRoll Agree
  Serial.print("checkRoll State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // landingAngle Value
  Serial.print("landingAngle Value: "); Serial.println(EEPROM.readFloat(address), 4);
  address += sizeof(float);

  // Lead Screw State
  Serial.print("Lead Screw State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // Camera Stepper State
  Serial.print("Camera Stepper State: "); Serial.println(EEPROM.readBool(address));
  address += sizeof(bool);

  // Camera Commands
  Serial.print("Camera Commands: ");

  const int ARRAY_SIZE = 30;
  const int STARTING_EEPROM_ADDRESS = address;

  int commandArray[ARRAY_SIZE];
  readIntArrayFromEEPROM(STARTING_EEPROM_ADDRESS, commandArray, ARRAY_SIZE);

  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    Serial.print(commandArray[i]); Serial.print(" ");
  }

  Serial.println(".");
  addressIndex++;
  
  // Interpret Radio String State
  Serial.print("Radio String State: "); Serial.println(EEPROM.readBool(addressIndex));
  

}

void loop() {}
