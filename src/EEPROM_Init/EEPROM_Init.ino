#include <EEPROM.h>

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11
#define TEMP_RESOLUTION_EEPROM_ADDR 12 //12
#define TEMP_OVERHEAT_WARNING_EEPROM_ADDR 13 //13-16
#define TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR 17 //17-20
#define TEMP_UNDERHEAT_WARNING_EEPROM_ADDR 21 //21-24
#define TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR 25 //25-28
#define CURRENT_WARNING_EERPOM_ADDR 29 //29-32
#define CURRENT_CRITICAL_EEPROM_ADDR 33 //33-36

float cellChargedVoltage = 4.2f;
float cellNominalVoltage = 3.7f;
float cellCriticalVoltage = 3.0f;

uint8_t tempResolution = 12;
float tempOverheatWarningThreshold = 50;
float tempOverheatCriticalThreshold = 60;
float tempUnderheatWarningThreshold = 10;
float tempUnderheatCriticalThreshold = 0;

float currentWarningThreshold = 25000;
float currentCriticalThreshold = 32000;

void setup() {
  EEPROM.put(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, cellChargedVoltage);
  EEPROM.put(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, cellNominalVoltage);
  EEPROM.put(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, cellCriticalVoltage);

  EEPROM.put(TEMP_RESOLUTION_EEPROM_ADDR, tempResolution);
  EEPROM.put(TEMP_OVERHEAT_WARNING_EEPROM_ADDR, tempOverheatWarningThreshold);
  EEPROM.put(TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR, tempOverheatCriticalThreshold);
  EEPROM.put(TEMP_UNDERHEAT_WARNING_EEPROM_ADDR, tempUnderheatWarningThreshold);
  EEPROM.put(TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR, tempUnderheatCriticalThreshold);
  
  EEPROM.put(CURRENT_WARNING_EERPOM_ADDR, currentWarningThreshold);
  EEPROM.put(CURRENT_CRITICAL_EEPROM_ADDR, currentCriticalThreshold);

  Serial.begin(115200);
  Serial.println("DONE!");
}

void loop() {

}
