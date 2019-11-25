#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//TODO allow address selection
#define WIRE_ID 8

#define LED_ONE_WIRE_PIN 6

#define TEMP_ONE_WIRE_PIN 2

//Pin to flash EEPROM to defaults when held high on boot
#define EEPROM_FLASH_PIN 3

//Address of the ADC for cells 0-3
#define ADC_0_ADDR 0x48

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0     //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4     //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8    //8-11
#define TEMP_RESOLUTION_EEPROM_ADDR 12         //12
#define TEMP_OVERHEAT_WARNING_EEPROM_ADDR 13   //13-16
#define TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR 17  //17-20
#define TEMP_UNDERHEAT_WARNING_EEPROM_ADDR 21  //21-24
#define TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR 25 //25-28
#define CURRENT_WARNING_EERPOM_ADDR 29         //29-32
#define CURRENT_CRITICAL_EEPROM_ADDR 33        //33-36
#define LED_BRIGHTNESS_EEPROM_ADDR 37          //37
#define REFRESH_RATE_EEPROM_ADDR 38            //38

Adafruit_NeoPixel status_LED(1, LED_ONE_WIRE_PIN, NEO_GRB);

OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature temperature_sensor(&oneWire);

#define ADC_GAIN GAIN_TWOTHIRDS
Adafruit_ADS1115 ads_0(ADC_0_ADDR);
const float VOLTAGE_DIVIDER_R1[] = {
    0.0f,
    1.764f,
    1.468f,
    6.650f};
const float VOLTAGE_DIVIDER_R2[] = {
    1.0f,
    3.290f,
    0.985f,
    3.270f};

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

struct MessageHeader
{
  MessageType message_type;
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

//Constant values to represent the current status of the battery
enum BatteryStatus
{
  OK = 0x00,
  WARNING = 0x01,
  CRITICAL = 0x02
};

Adafruit_INA219 ina219;

uint8_t refresh_rate;

uint8_t num_cells;

float cell_charged_voltage;
float cell_nominal_voltage;
float cell_critical_voltage;

uint8_t temperature_resolution;
float temperature_overheat_warning;
float temperature_overheat_critical;
float temperature_underheat_warning;
float temperature_underheat_critical;

float current_warning;
float current_critical;

uint8_t led_brightness;

byte request_type = 0x00;

void setup()
{
  Serial.begin(115200);

  status_LED.begin();

  pinMode(EEPROM_FLASH_PIN, INPUT);

  if (digitalRead(EEPROM_FLASH_PIN) == HIGH)
  {
    //Wait to confirm EEPROM flash
    for (int i = 0; i < 5; i++)
    {
      status_LED.fill(status_LED.Color(70, 0, 70));
      status_LED.show();
      delay(500);
      status_LED.fill(status_LED.Color(0, 0, 0));
      status_LED.show();
      delay(500);
    }

    //Check if pin is still high
    if (digitalRead(EEPROM_FLASH_PIN) == HIGH)
    {
      //Flash EEPROM to defaults
      setCellChargedVoltageEEPROM(4.2f);
      setCellNominalVoltageEEPROM(3.7f);
      setCellCriticalVoltageEEPROM(3.0f);

      setTempResolutionEEPROM(12);
      setTempOverheatWarningEEPROM(50);
      setTempOverheatCriticalEEPROM(60);
      setTempUnderheatWarningEEPROM(10);
      setTempUnderheatCriticalEEPROM(0);

      setCurrentWarningEEPROM(25000);
      setCurrentCriticalEEPROM(32000);

      setLEDBrightnessEEPROM(70);

      setRefreshRateEEPROM(1);

      //Indicate EEPROM flash is complete
      for (int i = 0; i < 3; i++)
      {
        status_LED.fill(status_LED.Color(0, 70, 0));
        status_LED.show();
        delay(100);
        status_LED.fill(status_LED.Color(0, 0, 0));
        status_LED.show();
        delay(100);
      }
    }
    else
    {
      //Pin is low, EEPROM flash canceled
      //Indicate EEPROM flash is canceled
      for (int i = 0; i < 3; i++)
      {
        status_LED.fill(status_LED.Color(70, 0, 0));
        status_LED.show();
        delay(100);
        status_LED.fill(status_LED.Color(0, 0, 0));
        status_LED.show();
        delay(100);
      }
    }
  }

  refresh_rate = getRefreshRateEEPROM();

  temperature_resolution = getTempResolutionEEPROM();
  temperature_overheat_critical = getTempOverheatCriticalEEPROM();
  temperature_underheat_critical = getTempUnderheatCriticalEEPROM();
  temperature_overheat_warning = getTempOverheatWarningEEPROM();
  temperature_underheat_warning = getTempUnderheatWarningEEPROM();

  current_warning = getCurrentWarningEEPROM();
  current_critical = getCurrentCriticalEEPROM();

  cell_charged_voltage = getCellChargedVoltageEEPROM();
  cell_nominal_voltage = getCellNominalVoltageEEPROM();
  cell_critical_voltage = getCellCriticalVoltageEEPROM();

  led_brightness = getLEDBrightnessEEPROM();

  Serial.println("Settings loaded from EEPROM:");
  Serial.print("Refresh Rate (Hz): ");
  Serial.println(refresh_rate);
  Serial.print("Cell Charged (V): ");
  Serial.println(cell_charged_voltage);
  Serial.print("Cell Nominal (V): ");
  Serial.println(cell_nominal_voltage);
  Serial.print("Cell Critical (V): ");
  Serial.println(cell_critical_voltage);
  Serial.print("Temp Resolution: ");
  Serial.println(temperature_resolution);
  Serial.print("Temp Overheat Warn (C): ");
  Serial.println(temperature_overheat_warning);
  Serial.print("Temp Overheat Crit (C): ");
  Serial.println(temperature_overheat_critical);
  Serial.print("Temp Underheat Warn (C): ");
  Serial.println(temperature_underheat_warning);
  Serial.print("Temp Underheat Crit (C): ");
  Serial.println(temperature_underheat_critical);
  Serial.print("Current Warn (mA): ");
  Serial.println(current_warning);
  Serial.print("Current Crit (mA): ");
  Serial.println(current_critical);
  Serial.print("LED Brightness: ");
  Serial.println(led_brightness);
  Serial.println();

  temperature_sensor.begin();
  temperature_sensor.setResolution(temperature_resolution);
  temperature_sensor.setWaitForConversion(false);
  temperature_sensor.setCheckForConversion(true);
  temperature_sensor.requestTemperatures();

  ads_0.begin();
  ads_0.setGain(ADC_GAIN);

  ina219.begin();

  //Detect the number of cells
  num_cells = 0;
  for (int i = 3; i > 0; i--)
  {
    float voltage = GetCellVoltage(i);
    if (voltage >= 2.0f)
    {
      num_cells = i + 1;
      break;
    }
  }
  Serial.print(num_cells);
  Serial.println(" cells detected.");

  Wire.begin(WIRE_ID);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  delay(2000);
}

Battery battery;
unsigned long prevPollTime = millis();
void loop()
{
  //Update temperature sensors
  if (temperature_sensor.isConversionComplete())
    temperature_sensor.requestTemperatures();

  if ((millis() - prevPollTime) > (1000 / refresh_rate))
  {
    prevPollTime = millis();

    battery = updateBattery();

    //    for (int i = 0; i < num_cells; i++)
    //    {
    //      Serial.print("Cell ");
    //      Serial.print(i);
    //      Serial.print(" Voltage: ");
    //      Serial.println(battery.cell_voltages[i], 4);
    //    }
    //
    //    Serial.print("Battery Voltage: ");
    //    Serial.println(battery.voltage, 4);
    //    Serial.print("Current (mA): ");
    //    Serial.println(battery.current, 4);
    //    Serial.print("Power (mW): ");
    //    Serial.println(battery.power, 4);
    //    Serial.print("Temperature (C): ");
    //    Serial.println(battery.temperature, 4);
    //    Serial.print("Errors: ");
    //    Serial.println(battery.errors, BIN);
    //    Serial.print("Status: ");
    //    Serial.println(battery.status);
    //    Serial.println();
  }
}

Battery updateBattery()
{
  Battery battery;

  for (int i = 0; i < num_cells; i++)
  {
    battery.cell_voltages[i] = GetCellVoltage(i);
  }

  //battery.voltage = GetBankVoltageAtIndex(num_cells - 1);
  //another option: use ina219 to calcuate load voltage
  battery.voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
  battery.current = ina219.getCurrent_mA();
  battery.power = ina219.getPower_mW();

  battery.temperature = temperature_sensor.getTempCByIndex(0);

  battery.errors = 0x00;

  //Detect if battery is present
  if (battery.voltage <= 1.0f)
  {
    battery.errors |= BATTERY_NOT_PRESENT;
  }
  else
  {
    for (int i = 0; i < num_cells; i++)
    {
      if (battery.cell_voltages[i] >= cell_charged_voltage)
      {
        battery.errors |= CELL_OVERVOLTAGE_FLAG;
      }

      if (battery.cell_voltages[i] <= cell_critical_voltage)
      {
        battery.errors |= CELL_CRITICAL_FLAG;
      }
      else if (battery.cell_voltages[i] <= cell_nominal_voltage)
      {
        battery.errors |= CELL_LOW_FLAG;
      }
    }
  }

  if (battery.temperature <= temperature_underheat_critical)
  {
    battery.errors |= TEMPERATURE_UNDERHEAT_CRITICAL_FLAG;
  }
  else if (battery.temperature <= temperature_underheat_warning)
  {
    battery.errors |= TEMPERATURE_UNDERHEAT_WARNING_FLAG;
  }

  if (battery.temperature >= temperature_overheat_critical)
  {
    battery.errors |= TEMPERATURE_OVERHEAT_CRITICAL_FLAG;
  }
  else if (battery.temperature >= temperature_overheat_warning)
  {
    battery.errors |= TEMPERATURE_OVERHEAT_WARNING_FLAG;
  }

  if (battery.current >= current_critical)
  {
    battery.errors |= CURRENT_CRITICAL_FLAG;
  }
  else if (battery.current >= current_warning)
  {
    battery.errors |= CURRENT_WARNING_FLAG;
  }

  battery.status = OK;
  if (
      battery.errors & CELL_OVERVOLTAGE_FLAG ||
      battery.errors & CELL_LOW_FLAG ||
      battery.errors & TEMPERATURE_UNDERHEAT_WARNING_FLAG ||
      battery.errors & TEMPERATURE_OVERHEAT_WARNING_FLAG ||
      battery.errors & CURRENT_WARNING_FLAG)
  {
    battery.status = WARNING;
  }

  if (
      battery.errors & CELL_CRITICAL_FLAG ||
      battery.errors & TEMPERATURE_UNDERHEAT_CRITICAL_FLAG ||
      battery.errors & TEMPERATURE_OVERHEAT_CRITICAL_FLAG ||
      battery.errors & CURRENT_CRITICAL_FLAG ||
      battery.errors & BATTERY_NOT_PRESENT)
  {
    battery.status = CRITICAL;
  }

  if (battery.status == OK)
  {
    status_LED.fill(status_LED.Color(0, led_brightness, 0));
  }
  else if (battery.status == WARNING)
  {
    status_LED.fill(status_LED.Color(led_brightness, led_brightness, 0));
  }
  else if (battery.status == CRITICAL)
  {
    status_LED.fill(status_LED.Color(led_brightness, 0, 0));
  }
  status_LED.show();

  return battery;
}

double GetCellVoltage(uint8_t Index)
{
  double CellVoltage = 0;
  if (Index == 0)
  {
    return GetBankVoltageAtIndex(Index);
  }
  else
  {
    return (GetBankVoltageAtIndex(Index) - GetBankVoltageAtIndex(Index - 1));
  }
  return -1;
}

float GetBankVoltageAtIndex(uint8_t Index)
{
  uint16_t analogValue = 0;
  if (Index >= 0 && Index <= 3)
  {
    analogValue = ads_0.readADC_SingleEnded(Index);
  }
  else
  {
    return -1;
  }

  return analogValue * ((0.1875f / 1000.0f) * ((VOLTAGE_DIVIDER_R1[Index] + VOLTAGE_DIVIDER_R2[Index]) / VOLTAGE_DIVIDER_R2[Index]));
}

void receiveEvent(int num_bytes)
{
  //Read MessageHeader
  byte header_data[sizeof(MessageHeader)];
  for (int i = 0; i < sizeof(MessageHeader); i++)
  {
    header_data[i] = Wire.read();
  }
  MessageHeader header;
  memcpy(&header, header_data, sizeof(MessageHeader));

  //Read remaining bytes
  uint8_t receive_bytes = num_bytes - sizeof(MessageHeader);
  byte receive_data[receive_bytes];
  for (int i = 0; i < receive_bytes; i++)
  {
    receive_data[i] = Wire.read();
  }

  switch (header.message_type)
  {
  case CELL_CHARGED_VOLTAGE:
    memcpy(&cell_charged_voltage, receive_data, sizeof(cell_charged_voltage));
    setCellChargedVoltageEEPROM(cell_charged_voltage);
    break;
  case CELL_NOMINAL_VOLTAGE:
    memcpy(&cell_nominal_voltage, receive_data, sizeof(cell_nominal_voltage));
    setCellNominalVoltageEEPROM(cell_nominal_voltage);
    break;
  case CELL_CRITICAL_VOLTAGE:
    memcpy(&cell_critical_voltage, receive_data, sizeof(cell_critical_voltage));
    setCellCriticalVoltageEEPROM(cell_critical_voltage);
    break;
  case TEMPERATURE_RESOLUTION:
    memcpy(&temperature_resolution, receive_data, sizeof(temperature_resolution));
    temperature_sensor.setResolution(temperature_resolution);
    setTempResolutionEEPROM(temperature_resolution);
    break;
  case TEMPERATURE_OVERHEAT_WARNING:
    memcpy(&temperature_overheat_critical, receive_data, sizeof(temperature_overheat_critical));
    setTempOverheatWarningEEPROM(temperature_overheat_critical);
    break;
  case TEMPERATURE_OVERHEAT_CRITICAL:
    memcpy(&temperature_overheat_warning, receive_data, sizeof(temperature_overheat_warning));
    setTempOverheatCriticalEEPROM(temperature_overheat_warning);
    break;
  case TEMPERATURE_UNDERHEAT_WARNING:
    memcpy(&temperature_underheat_warning, receive_data, sizeof(temperature_underheat_warning));
    setTempUnderheatWarningEEPROM(temperature_underheat_warning);
    break;
  case TEMPERATURE_UNDERHEAT_CRITICAL:
    memcpy(&temperature_underheat_critical, receive_data, sizeof(temperature_underheat_critical));
    setTempUnderheatCriticalEEPROM(temperature_underheat_critical);
    break;
  case CURRENT_WARNING:
    memcpy(&current_warning, receive_data, sizeof(current_warning));
    setCurrentWarningEEPROM(current_warning);
    break;
  case CURRENT_CRITICAL:
    memcpy(&current_critical, receive_data, sizeof(current_critical));
    setCurrentCriticalEEPROM(current_critical);
    break;
  case REQUEST_TYPE:
    memcpy(&request_type, receive_data, sizeof(request_type));
    break;
  case LED_BRIGHTNESS:
    memcpy(&led_brightness, receive_data, sizeof(led_brightness));
    setLEDBrightnessEEPROM(led_brightness);
    break;
  case REFRESH_RATE:
    memcpy(&refresh_rate, receive_data, sizeof(refresh_rate));
    setRefreshRateEEPROM(refresh_rate);
    break;
  }
}

void requestEvent()
{
  if (request_type == CELL_0_VOLTAGE)
  {
    float cell_0_voltage = battery.cell_voltages[0];
    sendData(&cell_0_voltage, sizeof(cell_0_voltage));
  }
  else if (request_type == CELL_1_VOLTAGE)
  {
    float cell_1_voltage = battery.cell_voltages[1];
    sendData(&cell_1_voltage, sizeof(cell_1_voltage));
  }
  else if (request_type == CELL_2_VOLTAGE)
  {
    float cell_2_voltage = battery.cell_voltages[2];
    sendData(&cell_2_voltage, sizeof(cell_2_voltage));
  }
  else if (request_type == CELL_3_VOLTAGE)
  {
    float cell_3_voltage = battery.cell_voltages[3];
    sendData(&cell_3_voltage, sizeof(cell_3_voltage));
  }
  else if (request_type == BATTERY_VOLTAGE)
  {
    float battery_voltage = battery.voltage;
    sendData(&battery_voltage, sizeof(battery_voltage));
  }
  else if (request_type == BATTERY_CURRENT)
  {
    float battery_current = battery.current;
    sendData(&battery_current, sizeof(battery_current));
  }
  else if (request_type == BATTERY_POWER)
  {
    float battery_power = battery.power;
    sendData(&battery_power, sizeof(battery_power));
  }
  else if (request_type == BATTERY_TEMPERATURE)
  {
    float battery_temperature = battery.temperature;
    sendData(&battery_temperature, sizeof(battery_temperature));
  }
  else if (request_type == BATTERY_ERROR)
  {
    uint16_t battery_errors = battery.errors;
    sendData(&battery_errors, sizeof(battery_errors));
  }
  else if (request_type == BATTERY_STATUS)
  {
    uint8_t battery_status = battery.status;
    sendData(&battery_status, sizeof(battery_status));
  }
  else if (request_type == CELL_CHARGED_VOLTAGE)
  {
    sendData(&cell_charged_voltage, sizeof(cell_charged_voltage));
  }
  else if (request_type == CELL_NOMINAL_VOLTAGE)
  {
    sendData(&cell_nominal_voltage, sizeof(cell_nominal_voltage));
  }
  else if (request_type == CELL_CRITICAL_VOLTAGE)
  {
    sendData(&cell_critical_voltage, sizeof(cell_critical_voltage));
  }
  else if (request_type == TEMPERATURE_RESOLUTION)
  {
    sendData(&temperature_resolution, sizeof(temperature_resolution));
  }
  else if (request_type == TEMPERATURE_OVERHEAT_WARNING)
  {
    sendData(&temperature_overheat_warning, sizeof(temperature_overheat_warning));
  }
  else if (request_type == TEMPERATURE_OVERHEAT_CRITICAL)
  {
    sendData(&temperature_overheat_critical, sizeof(temperature_overheat_critical));
  }
  else if (request_type == TEMPERATURE_UNDERHEAT_WARNING)
  {
    sendData(&temperature_underheat_warning, sizeof(temperature_underheat_warning));
  }
  else if (request_type == TEMPERATURE_UNDERHEAT_CRITICAL)
  {
    sendData(&temperature_underheat_critical, sizeof(temperature_underheat_critical));
  }
  else if (request_type == CURRENT_WARNING)
  {
    sendData(&current_warning, sizeof(current_warning));
  }
  else if (request_type == CURRENT_CRITICAL)
  {
    sendData(&current_critical, sizeof(current_critical));
  }
  else if (request_type == CELL_COUNT)
  {
    sendData(&num_cells, sizeof(num_cells));
  }
  else if (request_type == LED_BRIGHTNESS)
  {
    sendData(&led_brightness, sizeof(led_brightness));
  }
  else if (request_type == REFRESH_RATE)
  {
    sendData(&refresh_rate, sizeof(refresh_rate));
  }
}

//Function to send bytes to I2C master
void sendData(const void *data, uint8_t num_bytes)
{
  byte send_data[num_bytes];
  memcpy(send_data, data, num_bytes);
  Wire.write(send_data, num_bytes);
}

// ============ EEPROM Setters & Getters ============

//Cell charged voltage
void setCellChargedVoltageEEPROM(float voltage)
{
  EEPROM.put(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, voltage);
}
float getCellChargedVoltageEEPROM()
{
  float voltage = 0.00f;
  EEPROM.get(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

//Cell nominal voltage
void setCellNominalVoltageEEPROM(float voltage)
{
  EEPROM.put(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, voltage);
}
float getCellNominalVoltageEEPROM()
{
  float voltage = 0.00f;
  EEPROM.get(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

//Cell critical voltage
void setCellCriticalVoltageEEPROM(float voltage)
{
  EEPROM.put(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, voltage);
}
float getCellCriticalVoltageEEPROM()
{
  float voltage = 0.00f;
  EEPROM.get(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

//Temperature resolution
void setTempResolutionEEPROM(uint8_t resolution)
{
  EEPROM.put(TEMP_RESOLUTION_EEPROM_ADDR, resolution);
}
uint8_t getTempResolutionEEPROM()
{
  uint8_t resolution = 0;
  EEPROM.get(TEMP_RESOLUTION_EEPROM_ADDR, resolution);
  return resolution;
}

//Temperature overheat critical
void setTempOverheatCriticalEEPROM(float threshold)
{
  EEPROM.put(TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR, threshold);
}
float getTempOverheatCriticalEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}

//Temperature overheat warning
void setTempOverheatWarningEEPROM(float threshold)
{
  EEPROM.put(TEMP_OVERHEAT_WARNING_EEPROM_ADDR, threshold);
}
float getTempOverheatWarningEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(TEMP_OVERHEAT_WARNING_EEPROM_ADDR, threshold);
  return threshold;
}

//Temperature underheat critical
void setTempUnderheatCriticalEEPROM(float threshold)
{
  EEPROM.put(TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR, threshold);
}
float getTempUnderheatCriticalEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}

//Temperature underheat warning
void setTempUnderheatWarningEEPROM(float threshold)
{
  EEPROM.put(TEMP_UNDERHEAT_WARNING_EEPROM_ADDR, threshold);
}
float getTempUnderheatWarningEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(TEMP_UNDERHEAT_WARNING_EEPROM_ADDR, threshold);
  return threshold;
}

//Current warning
void setCurrentWarningEEPROM(float threshold)
{
  EEPROM.put(CURRENT_WARNING_EERPOM_ADDR, threshold);
}
float getCurrentWarningEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(CURRENT_WARNING_EERPOM_ADDR, threshold);
  return threshold;
}

//Current critical
void setCurrentCriticalEEPROM(float threshold)
{
  EEPROM.put(CURRENT_CRITICAL_EEPROM_ADDR, threshold);
}
float getCurrentCriticalEEPROM()
{
  float threshold = 0.0f;
  EEPROM.get(CURRENT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}

//LED brightness
void setLEDBrightnessEEPROM(uint8_t brightness)
{
  EEPROM.put(LED_BRIGHTNESS_EEPROM_ADDR, brightness);
}
uint8_t getLEDBrightnessEEPROM()
{
  uint8_t brightness = 0;
  EEPROM.get(LED_BRIGHTNESS_EEPROM_ADDR, brightness);
  return brightness;
}

//Refresh rate
void setRefreshRateEEPROM(uint8_t rate)
{
  EEPROM.put(REFRESH_RATE_EEPROM_ADDR, rate);
}
uint8_t getRefreshRateEEPROM()
{
  uint8_t rate = 0;
  EEPROM.get(REFRESH_RATE_EEPROM_ADDR, rate);
  return rate;
}