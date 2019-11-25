#include <Wire.h>
#include "BatteryManager.h"

#define NUM_CELLS 3
#define BATTERY_ADDRESS 8

BatteryManager battery;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  battery.begin(BATTERY_ADDRESS);
  //battery.printDetails();

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

  delay(2000);
}
