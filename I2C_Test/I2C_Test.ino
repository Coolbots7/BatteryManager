#include <Wire.h>

#define WIRE_ID 8

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

#define CELL_0_VOLTAGE_REGISTER 0x0

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(WIRE_ID);
  Wire.write(CELL_CHARGED_VOLTAGE_REGISTER);
  float charged_voltage = 4.2f;
  byte buffer[sizeof(charged_voltage)];
  memcpy(buffer, &charged_voltage, sizeof(buffer));
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();



  Wire.beginTransmission(WIRE_ID);
  Wire.write(CELL_NOMINAL_VOLTAGE_REGISTER);
  float nominal_voltage = 3.7f;
  buffer[sizeof(nominal_voltage)];
  memcpy(buffer, &nominal_voltage, sizeof(buffer));
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();



  Wire.beginTransmission(WIRE_ID);
  Wire.write(CELL_CRITICAL_VOLTAGE_REGISTER);
  float critical_voltage = 3.0f;
  buffer[sizeof(critical_voltage)];
  memcpy(buffer, &critical_voltage, sizeof(buffer));
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();
}

byte floatBuffer[sizeof(float)];
void loop() {

  //Set the register to the desired value to be read
//  Wire.beginTransmission(WIRE_ID);
//  byte reg = 16;
//  Wire.write(reg);
//  Wire.endTransmission();
  //Request desired value
//  Wire.requestFrom(WIRE_ID, sizeof(float));
//
//  delay(100);
//  if (Wire.available()) {
//    for (int i = 0; i < sizeof(floatBuffer); i++) {
//      floatBuffer[i] = Wire.read();
//    }
//
//    float val;
//    memcpy(&val, floatBuffer, sizeof(floatBuffer));
//    Serial.print("Got cell 0 voltage: "); Serial.println(val);
//  }
//
//  delay(1500);
}
