#include <Arduino.h>
#include <Wire.h>
#include "BatteryManager.h"

BatteryManager::BatteryManager()
{
}

BatteryManager::~BatteryManager()
{
}

void BatteryManager::begin(uint8_t address)
{
  wire_address = address;
}

void BatteryManager::printDetails()
{
  Serial.println("============ Settings loaded from EEPROM: ============");
  Serial.print("Cell Charged (V): ");
  Serial.println(getCellChargedVoltage());
  Serial.print("Cell Nominal (V): ");
  Serial.println(getCellNominalVoltage());
  Serial.print("Cell Critical (V): ");
  Serial.println(getCellCriticalVoltage());
  Serial.print("Temp Resolution (b): ");
  Serial.println(getTempResolution());
  Serial.print("Temp Overheat Warn (C): ");
  Serial.println(getOverheatWarningTemperature());
  Serial.print("Temp Overheat Crit (C): ");
  Serial.println(getOverheatCriticalTemperature());
  Serial.print("Temp Underheat Warn (C): ");
  Serial.println(getUnderheatWarningTemperature());
  Serial.print("Temp Underheat Crit (C): ");
  Serial.println(getUnderheatCriticalTemperature());
  Serial.print("Current Warn (mA): ");
  Serial.println(getCurrentWarning());
  Serial.print("Current Crit (mA): ");
  Serial.println(getCurrentCritical());
  Serial.println();
}

// ============ Get Battery Values ============
uint8_t BatteryManager::getCellCount()
{
  return getByte(CELL_COUNT);
}

float BatteryManager::getCellVoltage(uint8_t index)
{
  MessageType reg;
  switch (index)
  {
  case 0:
    reg = CELL_0_VOLTAGE;
    break;
  case 1:
    reg = CELL_1_VOLTAGE;
    break;
  case 2:
    reg = CELL_2_VOLTAGE;
    break;
  case 3:
    reg = CELL_3_VOLTAGE;
    break;
  default:
    return 0.0f;
  }

  return getFloat(reg);
}

float BatteryManager::getBatteryVoltage()
{
  return getFloat(BATTERY_VOLTAGE);
}

float BatteryManager::getBatteryCurrent()
{
  return getFloat(BATTERY_CURRENT);
}

float BatteryManager::getBatteryPower()
{
  return getFloat(BATTERY_POWER);
}

float BatteryManager::getBatteryTemperature()
{
  return getFloat(BATTERY_TEMPERATURE);
}

uint16_t BatteryManager::getBatteryErrors()
{
  return getUINT16(BATTERY_ERROR);
}

byte BatteryManager::getBatteryStatus()
{
  return getByte(BATTERY_STATUS);
}

// ============ Get EEPROM Settings ============
float BatteryManager::getCellChargedVoltage()
{
  return getFloat(CELL_CHARGED_VOLTAGE);
}
float BatteryManager::getCellNominalVoltage()
{
  return getFloat(CELL_NOMINAL_VOLTAGE);
}
float BatteryManager::getCellCriticalVoltage()
{
  return getFloat(CELL_CRITICAL_VOLTAGE);
}
byte BatteryManager::getTempResolution()
{
  return getByte(TEMPERATURE_RESOLUTION);
}
float BatteryManager::getOverheatWarningTemperature()
{
  return getFloat(TEMPERATURE_OVERHEAT_WARNING);
}
float BatteryManager::getOverheatCriticalTemperature()
{
  return getFloat(TEMPERATURE_OVERHEAT_CRITICAL);
}
float BatteryManager::getUnderheatWarningTemperature()
{
  return getFloat(TEMPERATURE_UNDERHEAT_WARNING);
}
float BatteryManager::getUnderheatCriticalTemperature()
{
  return getFloat(TEMPERATURE_UNDERHEAT_CRITICAL);
}
float BatteryManager::getCurrentWarning()
{
  return getFloat(CURRENT_WARNING);
}
float BatteryManager::getCurrentCritical()
{
  return getFloat(CURRENT_CRITICAL);
}
uint8_t BatteryManager::getLEDBrightness()
{
  return getByte(LED_BRIGHTNESS);
}
uint8_t BatteryManager::getRefreshRate()
{
  return getByte(REFRESH_RATE);
}

// ============ Set EEPROM Settings ============
void BatteryManager::setCellChargedVoltage(float voltage)
{
  sendData(CELL_CHARGED_VOLTAGE, &voltage, sizeof(voltage));
}
void BatteryManager::setCellNominalVoltage(float voltage)
{
  sendData(CELL_NOMINAL_VOLTAGE, &voltage, sizeof(voltage));
}
void BatteryManager::setCellCriticalVoltage(float voltage)
{
  sendData(CELL_CRITICAL_VOLTAGE, &voltage, sizeof(voltage));
}
void BatteryManager::setTempResolution(byte resolution)
{
  sendData(TEMPERATURE_RESOLUTION, &resolution, sizeof(resolution));
}
void BatteryManager::setOverheatWarningTemperature(float temperature)
{
  sendData(TEMPERATURE_OVERHEAT_WARNING, &temperature, sizeof(temperature));
}
void BatteryManager::setOverheatCriticalTemperature(float temperature)
{
  sendData(TEMPERATURE_OVERHEAT_CRITICAL, &temperature, sizeof(temperature));
}
void BatteryManager::setUnderheatWarningTemperature(float temperature)
{
  sendData(TEMPERATURE_UNDERHEAT_WARNING, &temperature, sizeof(temperature));
}
void BatteryManager::setUnderheatCriticalTemperature(float temperature)
{
  sendData(TEMPERATURE_UNDERHEAT_CRITICAL, &temperature, sizeof(temperature));
}
void BatteryManager::setCurrentWarning(float milliamps)
{
  sendData(CURRENT_WARNING, &milliamps, sizeof(milliamps));
}
void BatteryManager::setCurrentCritical(float milliamps)
{
  sendData(CURRENT_CRITICAL, &milliamps, sizeof(milliamps));
}
void BatteryManager::setLEDBrightness(uint8_t brightness)
{
  sendData(LED_BRIGHTNESS, &brightness, sizeof(brightness));
}
void BatteryManager::setRefreshRate(uint8_t refresh_rate)
{
  sendData(REFRESH_RATE, &refresh_rate, sizeof(refresh_rate));
}

void BatteryManager::getData(MessageType message_type, void *destination, uint8_t num_bytes)
{
  //Set the data being requested on the slave
  sendData(REQUEST_TYPE, &message_type, sizeof(byte));

  //Trigger request event on slave
  Wire.requestFrom(wire_address, num_bytes);
  byte received_bytes[num_bytes];
  for (int i = 0; i < num_bytes; i++)
  {
    received_bytes[i] = Wire.read();
  }
  memcpy(destination, received_bytes, num_bytes);
}

float BatteryManager::getFloat(MessageType message_type)
{
  float value;
  getData(message_type, &value, sizeof(value));
  return value;
}

byte BatteryManager::getByte(MessageType message_type)
{
  byte value;
  getData(message_type, &value, sizeof(value));
  return value;
}

uint16_t BatteryManager::getUINT16(MessageType message_type)
{
  uint16_t value;
  getData(message_type, &value, sizeof(value));
  return value;
}

void BatteryManager::sendData(MessageType message_type, const void *value, uint8_t num_bytes)
{
  byte send_bytes[sizeof(MessageHeader) + num_bytes];

  MessageHeader header;
  header.message_type = message_type;
  memcpy(send_bytes, &header, sizeof(MessageHeader));

  memcpy(send_bytes + sizeof(MessageHeader), value, num_bytes);

  Wire.beginTransmission(wire_address);
  Wire.write(send_bytes, sizeof(MessageHeader) + num_bytes);
  Wire.endTransmission();
}
