#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//TODO allow change update rate
#define RATE 1

//TODO allow address selection
#define WIRE_ID 8

//TODO autodetect number of cells
#define NUM_CELLS 3

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

Adafruit_NeoPixel statusLED(1, LED_ONE_WIRE_PIN, NEO_GRB);
//TODO move led brightness to EEPROM
//TODO allow change LED brightness
#define STATUS_LED_BRIGHTNESS 70

OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

#define ADC_GAIN GAIN_TWOTHIRDS
Adafruit_ADS1115 ads0(ADC_0_ADDR);
const float VOLTAGE_DIVIDER_R1[] = {
  0.0f,
  1.764f,
  1.468f,
  6.650f
};
const float VOLTAGE_DIVIDER_R2[] = {
  1.0f,
  3.290f,
  0.985f,
  3.270f
};

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
  BATTERY_STATUS = 0x8A
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
  CURRENT_CRITICAL_FLAG = 0x100
                          //IDEA battery disconected error
};

//Constant values to represent the current status of the battery
enum BatteryStatus
{
  OK = 0x00,
  WARNING = 0x01,
  CRITICAL = 0x02
};

Adafruit_INA219 ina219;

float cellChargedVoltage;
float cellNominalVoltage;
float cellCriticalVoltage;

uint8_t tempResolution;
float tempOverheatWarningThreshold;
float tempOverheatCriticalThreshold;
float tempUnderheatWarningThreshold;
float tempUnderheatCriticalThreshold;

float currentWarningThreshold;
float currentCriticalThreshold;

byte request_type = 0x00;

void setup()
{
  Serial.begin(115200);

  statusLED.begin();

  pinMode(EEPROM_FLASH_PIN, INPUT);

  if (digitalRead(EEPROM_FLASH_PIN) == HIGH)
  {
    //Wait to confirm EEPROM flash
    for (int i = 0; i < 5; i++)
    {
      statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, 0, STATUS_LED_BRIGHTNESS));
      statusLED.show();
      delay(500);
      statusLED.fill(statusLED.Color(0, 0, 0));
      statusLED.show();
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

      //Indicate EEPROM flash is complete
      for (int i = 0; i < 3; i++)
      {
        statusLED.fill(statusLED.Color(0, STATUS_LED_BRIGHTNESS, 0));
        statusLED.show();
        delay(100);
        statusLED.fill(statusLED.Color(0, 0, 0));
        statusLED.show();
        delay(100);
      }
    }
    else {
      //Pin is low, EEPROM flash canceled
      //Indicate EEPROM flash is canceled
      for (int i = 0; i < 3; i++)
      {
        statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, 0, 0));
        statusLED.show();
        delay(100);
        statusLED.fill(statusLED.Color(0, 0, 0));
        statusLED.show();
        delay(100);
      }
    }
  }

  Wire.begin(WIRE_ID);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  tempResolution = getTempResolutionEEPROM();
  tempOverheatCriticalThreshold = getTempOverheatCriticalEEPROM();
  tempUnderheatCriticalThreshold = getTempUnderheatCriticalEEPROM();
  tempOverheatWarningThreshold = getTempOverheatWarningEEPROM();
  tempUnderheatWarningThreshold = getTempUnderheatWarningEEPROM();

  currentWarningThreshold = getCurrentWarningEEPROM();
  currentCriticalThreshold = getCurrentCriticalEEPROM();

  cellChargedVoltage = getCellChargedVoltageEEPROM();
  cellNominalVoltage = getCellNominalVoltageEEPROM();
  cellCriticalVoltage = getCellCriticalVoltageEEPROM();

  tempSensor.begin();
  tempSensor.setResolution(tempResolution);
  tempSensor.setWaitForConversion(false);
  tempSensor.setCheckForConversion(true);
  tempSensor.requestTemperatures();

  ads0.begin();
  ads0.setGain(ADC_GAIN);

  ina219.begin();

  Serial.println("Settings loaded from EEPROM:");
  Serial.print("Cell Charged (V): ");
  Serial.println(cellChargedVoltage);
  Serial.print("Cell Nominal (V): ");
  Serial.println(cellNominalVoltage);
  Serial.print("Cell Critical (V): ");
  Serial.println(cellCriticalVoltage);
  Serial.print("Temp Resolution: ");
  Serial.println(tempResolution);
  Serial.print("Temp Overheat Warn (C): ");
  Serial.println(tempOverheatWarningThreshold);
  Serial.print("Temp Overheat Crit (C): ");
  Serial.println(tempOverheatCriticalThreshold);
  Serial.print("Temp Underheat Warn (C): ");
  Serial.println(tempUnderheatWarningThreshold);
  Serial.print("Temp Underheat Crit (C): ");
  Serial.println(tempUnderheatCriticalThreshold);
  Serial.print("Current Warn (mA): ");
  Serial.println(currentWarningThreshold);
  Serial.print("Current Crit (mA): ");
  Serial.println(currentCriticalThreshold);
  Serial.println();

  delay(2000);
}

Battery battery;
unsigned long prevPollTime = millis();
void loop()
{
  //Update temperature sensors
  if (tempSensor.isConversionComplete())
    tempSensor.requestTemperatures();

  if ((millis() - prevPollTime) > (1000 / RATE))
  {
    prevPollTime = millis();

    battery = updateBattery();

    //    for (int i = 0; i < NUM_CELLS; i++)
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

  //BUG when battery is disconnected cell 0 voltage is approx 0v but cell 1 voltge is ~18v, this causes both cell over and under voltage errors
  for (int i = 0; i < NUM_CELLS; i++)
  {
    battery.cell_voltages[i] = GetCellVoltage(i);
  }

  //battery.voltage = GetBankVoltageAtIndex(NUM_CELLS - 1);
  //another option: use ina219 to calcuate load voltage
  battery.voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);
  battery.current = ina219.getCurrent_mA();
  battery.power = ina219.getPower_mW();

  battery.temperature = tempSensor.getTempCByIndex(0);

  battery.errors = 0x00;
  for (int i = 0; i < NUM_CELLS; i++)
  {
    if (battery.cell_voltages[i] >= cellChargedVoltage)
    {
      battery.errors |= CELL_OVERVOLTAGE_FLAG;
    }

    if (battery.cell_voltages[i] <= cellCriticalVoltage)
    {
      battery.errors |= CELL_CRITICAL_FLAG;
    }
    else if (battery.cell_voltages[i] <= cellNominalVoltage)
    {
      battery.errors |= CELL_LOW_FLAG;
    }
  }

  if (battery.temperature <= tempUnderheatCriticalThreshold)
  {
    battery.errors |= TEMPERATURE_UNDERHEAT_CRITICAL_FLAG;
  }
  else if (battery.temperature <= tempUnderheatWarningThreshold)
  {
    battery.errors |= TEMPERATURE_UNDERHEAT_WARNING_FLAG;
  }

  if (battery.temperature >= tempOverheatCriticalThreshold)
  {
    battery.errors |= TEMPERATURE_OVERHEAT_CRITICAL_FLAG;
  }
  else if (battery.temperature >= tempOverheatWarningThreshold)
  {
    battery.errors |= TEMPERATURE_OVERHEAT_WARNING_FLAG;
  }

  if (battery.current >= currentCriticalThreshold)
  {
    battery.errors |= CURRENT_CRITICAL_FLAG;
  }
  else if (battery.current >= currentWarningThreshold)
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
    battery.errors & CURRENT_CRITICAL_FLAG)
  {
    battery.status = CRITICAL;
  }

  if (battery.status == OK)
  {
    statusLED.fill(statusLED.Color(0, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == WARNING)
  {
    statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == CRITICAL)
  {
    statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, 0, 0));
  }
  statusLED.show();

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
    analogValue = ads0.readADC_SingleEnded(Index);
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
      memcpy(&cellChargedVoltage, receive_data, sizeof(cellChargedVoltage));
      setCellChargedVoltageEEPROM(cellChargedVoltage);
      break;
    case CELL_NOMINAL_VOLTAGE:
      memcpy(&cellNominalVoltage, receive_data, sizeof(cellNominalVoltage));
      setCellNominalVoltageEEPROM(cellNominalVoltage);
      break;
    case CELL_CRITICAL_VOLTAGE:
      memcpy(&cellCriticalVoltage, receive_data, sizeof(cellCriticalVoltage));
      setCellCriticalVoltageEEPROM(cellCriticalVoltage);
      break;
    case TEMPERATURE_RESOLUTION:
      memcpy(&tempResolution, receive_data, sizeof(tempResolution));
      tempSensor.setResolution(tempResolution);
      setTempResolutionEEPROM(tempResolution);
      break;
    case TEMPERATURE_OVERHEAT_WARNING:
      memcpy(&tempOverheatCriticalThreshold, receive_data, sizeof(tempOverheatCriticalThreshold));
      setTempOverheatWarningEEPROM(tempOverheatCriticalThreshold);
      break;
    case TEMPERATURE_OVERHEAT_CRITICAL:
      memcpy(&tempOverheatWarningThreshold, receive_data, sizeof(tempOverheatWarningThreshold));
      setTempOverheatCriticalEEPROM(tempOverheatWarningThreshold);
      break;
    case TEMPERATURE_UNDERHEAT_WARNING:
      memcpy(&tempUnderheatWarningThreshold, receive_data, sizeof(tempUnderheatWarningThreshold));
      setTempUnderheatWarningEEPROM(tempUnderheatWarningThreshold);
      break;
    case TEMPERATURE_UNDERHEAT_CRITICAL:
      memcpy(&tempUnderheatCriticalThreshold, receive_data, sizeof(tempUnderheatCriticalThreshold));
      setTempUnderheatCriticalEEPROM(tempUnderheatCriticalThreshold);
      break;
    case CURRENT_WARNING:
      memcpy(&currentWarningThreshold, receive_data, sizeof(currentWarningThreshold));
      setCurrentWarningEEPROM(currentWarningThreshold);
      break;
    case CURRENT_CRITICAL:
      memcpy(&currentCriticalThreshold, receive_data, sizeof(currentCriticalThreshold));
      setCurrentCriticalEEPROM(currentCriticalThreshold);
      break;
    case REQUEST_TYPE:
      memcpy(&request_type, receive_data, sizeof(request_type));
      break;
  }
}

void requestEvent()
{
  if (request_type == CELL_0_VOLTAGE)
  {
    float cell0Voltage = battery.cell_voltages[0];
    sendData(&cell0Voltage, sizeof(cell0Voltage));
  }
  else if (request_type == CELL_1_VOLTAGE)
  {
    float cell1Voltage = battery.cell_voltages[1];
    sendData(&cell1Voltage, sizeof(cell1Voltage));
  }
  else if (request_type == CELL_2_VOLTAGE)
  {
    float cell2Voltage = battery.cell_voltages[2];
    sendData(&cell2Voltage, sizeof(cell2Voltage));
  }
  else if (request_type == CELL_3_VOLTAGE)
  {
    float cell3Voltage = battery.cell_voltages[3];
    sendData(&cell3Voltage, sizeof(cell3Voltage));
  }
  else if (request_type == BATTERY_VOLTAGE)
  {
    float batteryVoltage = battery.voltage;
    sendData(&batteryVoltage, sizeof(batteryVoltage));
  }
  else if (request_type == BATTERY_CURRENT)
  {
    float batteryCurrent = battery.current;
    sendData(&batteryCurrent, sizeof(batteryCurrent));
  }
  else if (request_type == BATTERY_POWER)
  {
    float batteryPower = battery.power;
    sendData(&batteryPower, sizeof(batteryPower));
  }
  else if (request_type == BATTERY_TEMPERATURE)
  {
    float batteryTemperature = battery.temperature;
    sendData(&batteryTemperature, sizeof(batteryTemperature));
  }
  else if (request_type == BATTERY_ERROR)
  {
    uint16_t batteryErrors = battery.errors;
    sendData(&batteryErrors, sizeof(batteryErrors));
  }
  else if (request_type == BATTERY_STATUS)
  {
    uint8_t batteryStatus = battery.status;
    sendData(&batteryStatus, sizeof(batteryStatus));
  }
  else if (request_type == CELL_CHARGED_VOLTAGE)
  {
    float cellChargedVoltage = getCellChargedVoltageEEPROM();
    sendData(&cellChargedVoltage, sizeof(cellChargedVoltage));
  }
  else if (request_type == CELL_NOMINAL_VOLTAGE)
  {
    float cellNominalVoltage = getCellNominalVoltageEEPROM();
    sendData(&cellNominalVoltage, sizeof(cellNominalVoltage));
  }
  else if (request_type == CELL_CRITICAL_VOLTAGE)
  {
    float cellCriticalVoltage = getCellCriticalVoltageEEPROM();
    sendData(&cellCriticalVoltage, sizeof(cellCriticalVoltage));
  }
  else if (request_type == TEMPERATURE_RESOLUTION)
  {
    uint8_t tempResolution = getTempResolutionEEPROM();
    sendData(&tempResolution, sizeof(tempResolution));
  }
  else if (request_type == TEMPERATURE_OVERHEAT_WARNING)
  {
    float tempOverheatWarning = getTempOverheatWarningEEPROM();
    sendData(&tempOverheatWarning, sizeof(tempOverheatWarning));
  }
  else if (request_type == TEMPERATURE_OVERHEAT_CRITICAL)
  {
    float tempOverheatCritical = getTempOverheatCriticalEEPROM();
    sendData(&tempOverheatCritical, sizeof(tempOverheatCritical));
  }
  else if (request_type == TEMPERATURE_UNDERHEAT_WARNING)
  {
    float tempUnderheatWarning = getTempUnderheatWarningEEPROM();
    sendData(&tempUnderheatWarning, sizeof(tempUnderheatWarning));
  }
  else if (request_type == TEMPERATURE_UNDERHEAT_CRITICAL)
  {
    float tempUnderheatCritical = getTempUnderheatCriticalEEPROM();
    sendData(&tempUnderheatCritical, sizeof(tempUnderheatCritical));
  }
  else if (request_type == CURRENT_WARNING)
  {
    float currentWarning = getCurrentWarningEEPROM();
    sendData(&currentWarning, sizeof(currentWarning));
  }
  else if (request_type == CURRENT_CRITICAL)
  {
    float currentCritical = getCurrentCriticalEEPROM();
    sendData(&currentCritical, sizeof(currentCritical));
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
