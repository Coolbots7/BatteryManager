#include <Wire.h>

#define WIRE_ID 8

#define NUM_CELLS 2

#define CELL_CHARGED_VOLTAGE_REGISTER 0x01
#define CELL_NOMINAL_VOLTAGE_REGISTER 0x02
#define CELL_CRITICAL_VOLTAGE_REGISTER 0x03
#define TEMP_RESOLUTION_REGISTER 0x04
#define TEMP_OVERHEAT_WARNING_REGISTER 0x05
#define TEMP_OVERHEAT_CRITICAL_REGISTER 0x06
#define TEMP_UNDERHEAT_WARNING_REGISTER 0x07
#define TEMP_UNDERHEAT_CRITICAL_REGISTER 0x08
#define CURRENT_WARNING_REGISTER 0x09
#define CURRENT_CRITICAL_REGISTER 0x0A

#define CELL_0_VOLTAGE_REGISTER 0x10
#define CELL_1_VOLTAGE_REGISTER 0x20
#define CELL_2_VOLTAGE_REGISTER 0x30
#define CELL_3_VOLTAGE_REGISTER 0x40
#define CELL_4_VOLTAGE_REGISTER 0x50
#define CELL_5_VOLTAGE_REGISTER 0x60
#define CELL_6_VOLTAGE_REGISTER 0x70
#define CELL_7_VOLTAGE_REGISTER 0x80
#define BATTERY_VOLTAGE_REGISTER 0x90
#define BATTERY_CURRENT_REGISTER 0xA0
#define BATTERY_POWER_REGISTER 0xB0
#define BATTERY_TEMPERATURE_REGISTER 0xC0
#define BATTERY_ERROR_REGISTER 0xD0
#define BATTERY_STATUS_REGISTER 0xE0

float cellVoltages[NUM_CELLS];
float voltage;
float current;
float power;
float temperature;
uint16_t errors;
byte status;

byte floatBuffer[sizeof(float)];
byte uint16Buffer[sizeof(uint16_t)];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  //  Wire.beginTransmission(WIRE_ID);
  //  Wire.write(CELL_CHARGED_VOLTAGE_REGISTER);
  //  float charged_voltage = 4.2f;
  //  byte buffer[sizeof(charged_voltage)];
  //  memcpy(buffer, &charged_voltage, sizeof(buffer));
  //  Wire.write(buffer, sizeof(buffer));
  //  Wire.endTransmission();
  //
  //
  //
  //  Wire.beginTransmission(WIRE_ID);
  //  Wire.write(CELL_NOMINAL_VOLTAGE_REGISTER);
  //  float nominal_voltage = 3.7f;
  //  buffer[sizeof(nominal_voltage)];
  //  memcpy(buffer, &nominal_voltage, sizeof(buffer));
  //  Wire.write(buffer, sizeof(buffer));
  //  Wire.endTransmission();
  //
  //
  //
  //  Wire.beginTransmission(WIRE_ID);
  //  Wire.write(CELL_CRITICAL_VOLTAGE_REGISTER);
  //  float critical_voltage = 3.0f;
  //  buffer[sizeof(critical_voltage)];
  //  memcpy(buffer, &critical_voltage, sizeof(buffer));
  //  Wire.write(buffer, sizeof(buffer));
  //  Wire.endTransmission();
  //  delay(2000);

  
  Serial.println("Settings loaded from EEPROM:");
  Serial.print("Cell Charged (V): "); Serial.println(getCellChargedVoltage());
  Serial.print("Cell Nominal (V): "); Serial.println(getCellNominalVoltage());
  Serial.print("Cell Critical (V): "); Serial.println(getCellCriticalVoltage());
  Serial.print("Temp Resolution: "); Serial.println(getTempResolution());
  Serial.print("Temp Overheat Warn (C): "); Serial.println(getOverheatWarningTemperature());
  Serial.print("Temp Overheat Crit (C): "); Serial.println(getOverheatCriticalTemperature());
  Serial.print("Temp Underheat Warn (C): "); Serial.println(getUnderheatWarningTemperature());
  Serial.print("Temp Underheat Crit (C): "); Serial.println(getUnderheatCriticalTemperature());
  Serial.print("Current Warn (mA): "); Serial.println(getCurrentWarning());
  Serial.print("Current Crit (mA): "); Serial.println(getCurrentCritical());
  Serial.println();

  delay(2000);
}

void loop() {

  for (int i = 0; i < NUM_CELLS; i++) {
    cellVoltages[i] = getCellVoltage(i);
  }

  voltage = getBatteryVoltage();
  current = getBatteryCurrent();
  power = getBatteryPower();
  temperature = getBatteryTemperature();
  errors = getBatteryErrors();
  status = getBatteryStatus();


  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.print("Cell "); Serial.print(i); Serial.print(" Voltage: " ); Serial.println(cellVoltages[i], 4);
  }

  Serial.print("Battery Voltage: "); Serial.println(voltage, 4);
  Serial.print("Current (mA): "); Serial.println(current, 4);
  Serial.print("Power (mW): "); Serial.println(power, 4);
  Serial.print("Temperature (C): "); Serial.println(temperature, 4);
  Serial.print("Errors: "); Serial.println(errors, BIN);
  Serial.print("Status: "); Serial.println(status);
  Serial.println();

  delay(500);
}

float getCellVoltage(uint8_t cell) {
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

  return getFloat(WIRE_ID, reg);
}

float getBatteryVoltage() {
  return getFloat(WIRE_ID, BATTERY_VOLTAGE_REGISTER);
}

float getBatteryCurrent() {
  return getFloat(WIRE_ID, BATTERY_CURRENT_REGISTER);
}

float getBatteryPower() {
  return getFloat(WIRE_ID, BATTERY_POWER_REGISTER);
}

float getBatteryTemperature() {
  return getFloat(WIRE_ID, BATTERY_TEMPERATURE_REGISTER);
}

uint16_t getBatteryErrors() {
  return getuint16(WIRE_ID, BATTERY_ERROR_REGISTER);
}

byte getBatteryStatus() {
  return getByte(WIRE_ID, BATTERY_STATUS_REGISTER);
}

float getCellChargedVoltage() {
  return getFloat(WIRE_ID, CELL_CHARGED_VOLTAGE_REGISTER);
}
float getCellNominalVoltage() {
  return getFloat(WIRE_ID, CELL_NOMINAL_VOLTAGE_REGISTER);
}
float getCellCriticalVoltage() {
  return getFloat(WIRE_ID, CELL_CRITICAL_VOLTAGE_REGISTER);
}
byte getTempResolution() {
  return getByte(WIRE_ID, TEMP_RESOLUTION_REGISTER);
}
float getOverheatWarningTemperature() {
  return getFloat(WIRE_ID, TEMP_OVERHEAT_WARNING_REGISTER);
}
float getOverheatCriticalTemperature() {
  return getFloat(WIRE_ID, TEMP_OVERHEAT_CRITICAL_REGISTER);
}
float getUnderheatWarningTemperature() {
  return getFloat(WIRE_ID, TEMP_UNDERHEAT_WARNING_REGISTER);
}
float getUnderheatCriticalTemperature() {
  return getFloat(WIRE_ID, TEMP_UNDERHEAT_CRITICAL_REGISTER);
}
float getCurrentWarning() {
  return getFloat(WIRE_ID, CURRENT_WARNING_REGISTER);
}
float getCurrentCritical() {
  return getFloat(WIRE_ID, CURRENT_CRITICAL_REGISTER);
}

float getFloat(byte id, byte reg) {
  float val;

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(float));
  if (Wire.available()) {
    for (int i = 0; i < sizeof(floatBuffer); i++) {
      floatBuffer[i] = Wire.read();
    }
    memcpy(&val, floatBuffer, sizeof(floatBuffer));
  }

  return val;
}

uint16_t getuint16(byte id, byte reg) {
  float val;

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(uint16_t));
  if (Wire.available()) {
    for (int i = 0; i < sizeof(uint16Buffer); i++) {
      uint16Buffer[i] = Wire.read();
    }
    memcpy(&val, uint16Buffer, sizeof(uint16Buffer));
  }

  return val;
}

byte getByte(byte id, byte reg) {
  byte val;

  //Set the register to the desired value to be read
  Wire.beginTransmission(id);
  //TODO replace value with declared const
  Wire.write(reg);
  Wire.endTransmission();
  //Request desired value
  Wire.requestFrom(id, sizeof(float));
  if (Wire.available()) {
    val = Wire.read();
  }

  return val;
}
