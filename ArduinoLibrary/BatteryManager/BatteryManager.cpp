#include <Arduino.h>
#include <Wire.h>
#include <BatteryManager.h>

BatteryManager::BatteryManager(uint8_t addr) {
     i2caddress = addr;
}

BatteryManager::~BatteryManager() {

}

void BatteryManager::begin() {
     Wire.begin();
}

float BatteryManager::getCellVoltage(uint8_t cell) {
  byte reg;
  switch (cell) {
    case 0:
      reg = CELL_0_VOLTAGE_REGISTER;
      break;
    case 1:
      reg = CELL_1_VOLTAGE_REGISTER;
      break;
    case 2:
      reg = CELL_2_VOLTAGE_REGISTER;
      break;
    case 3:
      reg = CELL_3_VOLTAGE_REGISTER;
      break;
    case 4:
      reg = CELL_4_VOLTAGE_REGISTER;
      break;
    case 5:
      reg = CELL_5_VOLTAGE_REGISTER;
      break;
    case 6:
      reg = CELL_6_VOLTAGE_REGISTER;
      break;
    case 7:
      reg = CELL_7_VOLTAGE_REGISTER;
      break;
    default:
      break;
  }

  return getFloat(i2caddress, reg);
}

float BatteryManager::getBatteryVoltage() {
  return getFloat(i2caddress, BATTERY_VOLTAGE_REGISTER);
}

float BatteryManager::getBatteryCurrent() {
  return getFloat(i2caddress, BATTERY_CURRENT_REGISTER);
}

float BatteryManager::getBatteryPower() {
  return getFloat(i2caddress, BATTERY_POWER_REGISTER);
}

float BatteryManager::getBatteryTemperature() {
  return getFloat(i2caddress, BATTERY_TEMPERATURE_REGISTER);
}

uint16_t BatteryManager::getBatteryErrors() {
  return getuint16(i2caddress, BATTERY_ERROR_REGISTER);
}

byte BatteryManager::getBatteryStatus() {
  return getByte(i2caddress, BATTERY_STATUS_REGISTER);
}

float BatteryManager::getCellChargedVoltage() {
  return getFloat(i2caddress, CELL_CHARGED_VOLTAGE_REGISTER);
}
float BatteryManager::getCellNominalVoltage() {
  return getFloat(i2caddress, CELL_NOMINAL_VOLTAGE_REGISTER);
}
float BatteryManager::getCellCriticalVoltage() {
  return getFloat(i2caddress, CELL_CRITICAL_VOLTAGE_REGISTER);
}
byte BatteryManager::getTempResolution() {
  return getByte(i2caddress, TEMP_RESOLUTION_REGISTER);
}
float BatteryManager::getOverheatWarningTemperature() {
  return getFloat(i2caddress, TEMP_OVERHEAT_WARNING_REGISTER);
}
float BatteryManager::getOverheatCriticalTemperature() {
  return getFloat(i2caddress, TEMP_OVERHEAT_CRITICAL_REGISTER);
}
float BatteryManager::getUnderheatWarningTemperature() {
  return getFloat(i2caddress, TEMP_UNDERHEAT_WARNING_REGISTER);
}
float BatteryManager::getUnderheatCriticalTemperature() {
  return getFloat(i2caddress, TEMP_UNDERHEAT_CRITICAL_REGISTER);
}
float BatteryManager::getCurrentWarning() {
  return getFloat(i2caddress, CURRENT_WARNING_REGISTER);
}
float BatteryManager::getCurrentCritical() {
  return getFloat(i2caddress, CURRENT_CRITICAL_REGISTER);
}

float BatteryManager::getFloat(uint8_t id, byte reg) {
  float val;

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(float));


  byte floatBuffer[sizeof(float)];
  if (Wire.available()) {
    for (int i = 0; i < sizeof(floatBuffer); i++) {
      floatBuffer[i] = Wire.read();
    }
    memcpy(&val, floatBuffer, sizeof(floatBuffer));
  }

  return val;
}

uint16_t BatteryManager::getuint16(uint8_t id, byte reg) {
  float val;

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(uint16_t));
  return ((Wire.read()<<8) | Wire.read());
}

byte BatteryManager::getByte(uint8_t id, byte reg) {

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(float));
  return Wire.read();
}
