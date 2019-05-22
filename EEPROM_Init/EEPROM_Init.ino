#include <EEPROM.h>

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11

float cellChargedVoltage = 4.2f;
float cellNominalVoltage = 3.7f;
float cellCriticalVoltage = 3.0f;

void setup() {
  EEPROM.put(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, cellChargedVoltage);
  EEPROM.put(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, cellNominalVoltage);
  EEPROM.put(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, cellCriticalVoltage);

  Serial.begin(115200);
  Serial.println("DONE!");
}

void loop() {

}
