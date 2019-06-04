#include <Wire.h>
#include <BatteryManager.h>

#define NUM_CELLS 2
BatteryManager battery(8);

void setup() {
  Serial.begin(115200);

  battery.begin();
  
  Serial.println("Settings loaded from EEPROM:");
  Serial.print("Cell Charged (V): "); Serial.println(battery.getCellChargedVoltage());
  Serial.print("Cell Nominal (V): "); Serial.println(battery.getCellNominalVoltage());
  Serial.print("Cell Critical (V): "); Serial.println(battery.getCellCriticalVoltage());
  Serial.print("Temp Resolution: "); Serial.println(battery.getTempResolution());
  Serial.print("Temp Overheat Warn (C): "); Serial.println(battery.getOverheatWarningTemperature());
  Serial.print("Temp Overheat Crit (C): "); Serial.println(battery.getOverheatCriticalTemperature());
  Serial.print("Temp Underheat Warn (C): "); Serial.println(battery.getUnderheatWarningTemperature());
  Serial.print("Temp Underheat Crit (C): "); Serial.println(battery.getUnderheatCriticalTemperature());
  Serial.print("Current Warn (mA): "); Serial.println(battery.getCurrentWarning());
  Serial.print("Current Crit (mA): "); Serial.println(battery.getCurrentCritical());
  
  delay(1000);
}

void loop() {

  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.print("Cell "); Serial.print(i); Serial.print(" Voltage: " ); Serial.println(battery.getCellVoltage(i), 4);
  }

  Serial.print("Battery Voltage: "); Serial.println(battery.getBatteryVoltage(), 4);
  Serial.print("Current (mA): "); Serial.println(battery.getBatteryCurrent(), 4);
  Serial.print("Power (mW): "); Serial.println(battery.getBatteryPower(), 4);
  Serial.print("Temperature (C): "); Serial.println(battery.getBatteryTemperature(), 4);
  Serial.print("Errors: "); Serial.println(battery.getBatteryErrors(), BIN);
  Serial.print("Status: "); Serial.println(battery.getBatteryStatus());
  Serial.println();

  delay(500);

}
