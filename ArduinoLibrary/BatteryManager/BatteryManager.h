#include <Arduino.h>

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

 class BatteryManager {
 public:
      BatteryManager(byte addr);
      ~BatteryManager();

      void begin();

      float getCellVoltage(uint8_t cell);
      float getBatteryVoltage();
      float getBatteryCurrent();
      float getBatteryPower();
      float getBatteryTemperature();
      uint16_t getBatteryErrors();
      byte getBatteryStatus();

      float getCellChargedVoltage();
      float getCellNominalVoltage();
      float getCellCriticalVoltage();
      byte getTempResolution();
      float getOverheatWarningTemperature();
      float getOverheatCriticalTemperature();
      float getUnderheatWarningTemperature();
      float getUnderheatCriticalTemperature();
      float getCurrentWarning();
      float getCurrentCritical();


       void setCellChargedVoltage(float);
       void setCellNominalVoltage(float);
       void setCellCriticalVoltage(float);
       void setTempResolution(byte);
       void setOverheatWarningTemperature(float);
       void setOverheatCriticalTemperature(float);
       void setUnderheatWarningTemperature(float);
       void setUnderheatCriticalTemperature(float);
       void setCurrentWarning(float);
       void setCurrentCritical(float);
 private:
      uint8_t i2caddress;
      void writeRegister(byte reg, const void* object, byte size);
      float getFloat(byte reg);
      uint16_t getuint16(byte reg);
      byte getByte(byte reg);
};
