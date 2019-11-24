#include <Wire.h>
#include <BatteryManager.h>

#define BATTERY_ADDRESS 8
BatteryManager battery;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  battery.begin(BATTERY_ADDRESS);
  battery.printDetails();

  //Set battery settings
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
  
  battery.printDetails();
}

void loop() {
  delay(100);
}
