#include <Wire.h>
#include <BatteryManager.h>

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

  battery.setCellChargedVoltage(4.2f);
  battery.setCellNominalVoltage(3.7f);
  battery.setCellCriticalVoltage(3.0f);
  battery.setTempResolution(12);
  battery.setOverheatWarningTemperature(50.0f);
  battery.setOverheatCriticalTemperature(60.0f);
  battery.setUnderheatWarningTemperature(10.0f);
  battery.setUnderheatCriticalTemperature(0.0f);
  battery.setCurrentWarning(900.0f);
  battery.setCurrentCritical(1000.0f);
  
  Serial.println("UPDATED Settings loaded from EEPROM:");
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
}

void loop() {
  // put your main code here, to run repeatedly:

}
