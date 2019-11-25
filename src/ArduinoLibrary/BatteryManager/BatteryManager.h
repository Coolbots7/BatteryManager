#include <Arduino.h>

#define REQUEST_TIMEOUT 100

struct Battery
{
  float cell_voltages[4];
  float voltage;
  float current;
  float power;
  float temperature;
  uint16_t errors;
  uint8_t status;
};

//Constant values to represent the current status of the battery
enum BatteryStatus
{
  OK = 0x00,
  WARNING = 0x01,
  CRITICAL = 0x02
};

enum MessageType
{
//Constants to represent the intent of the value being sent to the battery manager
  CELL_CHARGED_VOLTAGE = 0x01,
  CELL_NOMINAL_VOLTAGE = 0x02,
  CELL_CRITICAL_VOLTAGE = 0x03,
  TEMPERATURE_RESOLUTION = 0x04,
  TEMPERATURE_OVERHEAT_WARNING = 0x05,
  TEMPERATURE_OVERHEAT_CRITICAL = 0x06,
  TEMPERATURE_UNDERHEAT_WARNING = 0x07,
  TEMPERATURE_UNDERHEAT_CRITICAL = 0x08,
  CURRENT_WARNING = 0x09,
  CURRENT_CRITICAL = 0x0A,
  REQUEST_TYPE = 0x0B,
  LED_BRIGHTNESS = 0x0C,
  REFRESH_RATE = 0x0D,
  //Constants to represent the intended value being requested from the battery manager
  CELL_0_VOLTAGE = 0x81,
  CELL_1_VOLTAGE = 0x82,
  CELL_2_VOLTAGE = 0x83,
  CELL_3_VOLTAGE = 0x84,
  BATTERY_VOLTAGE = 0x85,
  BATTERY_CURRENT = 0x86,
  BATTERY_POWER = 0x87,
  BATTERY_TEMPERATURE = 0x88,
  BATTERY_ERROR = 0x89,
  BATTERY_STATUS = 0x8A,
  CELL_COUNT = 0x8B
};

//Constant bit flags used to represent battery errors
enum BatteryError
{
  CELL_OVERVOLTAGE_FLAG = 0x01,
  CELL_LOW_FLAG = 0x02,
  CELL_CRITICAL_FLAG = 0x04,
  TEMPERATURE_OVERHEAT_WARNING_FLAG = 0x08,
  TEMPERATURE_OVERHEAT_CRITICAL_FLAG = 0x10,
  TEMPERATURE_UNDERHEAT_WARNING_FLAG = 0x20,
  TEMPERATURE_UNDERHEAT_CRITICAL_FLAG = 0x40,
  CURRENT_WARNING_FLAG = 0x80,
  CURRENT_CRITICAL_FLAG = 0x100,
  BATTERY_NOT_PRESENT = 0x200
};

struct MessageHeader
{
  MessageType message_type;
};

class BatteryManager
{
  public:
    BatteryManager();
    ~BatteryManager();

    void begin(uint8_t address);

    void printDetails();

    uint8_t getCellCount();
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
    uint8_t getLEDBrightness();
    uint8_t getRefreshRate();

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
    void setLEDBrightness(uint8_t);
    void setRefreshRate(uint8_t);

  private:
    uint8_t wire_address;

    void sendData(MessageType message_type, const void *value, uint8_t num_bytes);
    void getData(MessageType message_type, void *destination, uint8_t num_bytes);
    float getFloat(MessageType message_type);
    byte getByte(MessageType message_type);
    uint16_t getUINT16(MessageType message_type);
};
