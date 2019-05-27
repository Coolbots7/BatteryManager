#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define RATE 10

#define WIRE_ID 8

//TODO move num cells to EEPROM
//TODO autodetect number of cells
#define NUM_CELLS 2

#define LED_ONE_WIRE_PIN 6

#define TEMP_ONE_WIRE_PIN 2

//TODO add over current threshold'pu

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11
#define TEMP_RESOLUTION_EEPROM_ADDR 12 //12
#define TEMP_OVERHEAT_WARNING_EEPROM_ADDR 13 //13-16
#define TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR 17 //17-20
#define TEMP_UNDERHEAT_WARNING_EEPROM_ADDR 21 //21-24
#define TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR 25 //25-28

Adafruit_NeoPixel statusLED(1, LED_ONE_WIRE_PIN, NEO_GRB);
//TODO move led brightness to EEPROM
#define STATUS_LED_BRIGHTNESS 127

OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

#define ADC_GAIN GAIN_TWOTHIRDS
#define ADC_0_ADDR 0x48
#define ADC_1_ADDR 0x49
Adafruit_ADS1115 ads0(ADC_0_ADDR);
Adafruit_ADS1115 ads1(ADC_1_ADDR);
const float VOLTAGE_DIVIDER_R1[] = {
  0.0f,
  1.764f,
  1.468f,
  6.650f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
};

const float VOLTAGE_DIVIDER_R2[] = {
  1.0f,
  3.290f,
  0.985f,
  3.270f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
};

Adafruit_INA219 ina219;

struct Battery {
  float cellVoltages[NUM_CELLS];
  float voltage;
  float current;
  float power;
  float temperature;
  byte status;
};

//TODO make error code flags

float cellChargedVoltage;
float cellNominalVoltage;
float cellCriticalVoltage;

uint8_t tempResolution;
float tempOverheatWarningThreshold;
float tempOverheatCriticalThreshold;
float tempUnderheatWarningThreshold;
float tempUnderheatCriticalThreshold;

void setup() {
  Serial.begin(115200);
  Wire.begin(WIRE_ID);
  Wire.onRequest(wireRequest);

  tempResolution = getTempResolutionEEPROM();
  tempOverheatCriticalThreshold = getTempOverheatCriticalEEPROM();
  tempUnderheatCriticalThreshold = getTempUnderheatCriticalEEPROM();
  tempOverheatWarningThreshold = getTempOverheatWarningEEPROM();
  tempUnderheatWarningThreshold = getTempUnderheatWarningEEPROM();

  tempSensor.begin();
  tempSensor.setResolution(tempResolution);
  tempSensor.setWaitForConversion(false);
  tempSensor.setCheckForConversion(true);
  tempSensor.requestTemperatures();

  ads0.begin();
  ads0.setGain(ADC_GAIN);

  ads1.begin();
  ads1.setGain(ADC_GAIN);

  ina219.begin();

  statusLED.begin();

  cellChargedVoltage = getCellChargedVoltageEEPROM();
  cellNominalVoltage = getCellNominalVoltageEEPROM();
  cellCriticalVoltage = getCellCriticalVoltageEEPROM();
}

unsigned long prevPollTime = millis();
void loop() {
  //Update temperature sensors
  if (tempSensor.isConversionComplete())
    tempSensor.requestTemperatures();

  if ((millis() - prevPollTime) > (1000 / RATE)) {
    prevPollTime = millis();

    Battery battery = updateBattery();

    for (int i = 0; i < NUM_CELLS; i++) {
      Serial.print("Cell "); Serial.print(i); Serial.print(" Voltage: " );
      Serial.println(battery.cellVoltages[i], 4);
    }

    Serial.print("Battery Voltage: ");
    Serial.println(battery.voltage, 4);
    Serial.print("Current (mA): ");
    Serial.println(battery.current, 4);
    Serial.print("Power (mW): ");
    Serial.println(battery.power, 4);
    Serial.print("Temperature (C): ");
    Serial.println(battery.temperature, 4);
    Serial.print("Status: ");
    Serial.println(battery.status);
    Serial.println();
  }

}

void wireRequest() {
  Serial.println("Sending Battery info");

  Battery battery = updateBattery();

  byte buffer[sizeof(battery)];
  memcpy(buffer, &battery, sizeof(battery));
  Wire.write(buffer, sizeof(buffer));
}

Battery updateBattery() {
  Battery battery;

  for (int i = 0; i < NUM_CELLS; i++) {
    battery.cellVoltages[i] = GetCellVoltage(i);
  }

  battery.voltage = GetBankVoltageAtIndex(NUM_CELLS - 1);
  //another option: use ina219 to calcuate load voltage
  //battery.voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);

  battery.current = ina219.getCurrent_mA();
  battery.power = ina219.getPower_mW();
  battery.temperature = tempSensor.getTempCByIndex(0);

  battery.status = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.cellVoltages[i] > cellChargedVoltage) {
      battery.status = 1;
    }
  }
  

  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.cellVoltages[i] <= cellNominalVoltage) {
      battery.status = 2;
    }
  }

  if (battery.temperature  <= tempUnderheatWarningThreshold) {
    battery.status = 2;
  }

  if (battery.temperature >= tempOverheatWarningThreshold) {
    battery.status = 2;
  }


  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.cellVoltages[i] <= cellCriticalVoltage) {
      battery.status = 3;
    }
  }

  if (battery.temperature >= tempOverheatCriticalThreshold) {
    battery.status = 3;
  }

  if (battery.temperature <= tempUnderheatCriticalThreshold) {
    battery.status = 3;
  }


  if (battery.status == 0) {
    statusLED.fill(statusLED.Color(0, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == 2 || battery.status == 1) {
    statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == 3) {
    statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, 0, 0));
  }
  statusLED.show();

  return battery;
}

double GetCellVoltage(uint8_t Index) {
  double CellVoltage = 0;
  if (Index == 0) {
    return GetBankVoltageAtIndex(Index);
  }
  else {
    return (GetBankVoltageAtIndex(Index) - GetBankVoltageAtIndex(Index - 1));
  }
  return -1;
}

float GetBankVoltageAtIndex(uint8_t Index) {
  uint16_t analogValue = 0;
  if (Index >= 0 && Index <= 3) {
    analogValue = ads0.readADC_SingleEnded(Index);
  }
  else if (Index > 3 && Index <= 5) {
    analogValue = ads1.readADC_SingleEnded(Index - 4);
  }
  else {
    return -1;
  }

  return analogValue  * ((0.1875f / 1000.0f) * ((VOLTAGE_DIVIDER_R1[Index] + VOLTAGE_DIVIDER_R2[Index]) / VOLTAGE_DIVIDER_R2[Index]));
}

void setCellChargedVoltageEEPROM(float voltage) {
  EEPROM.put(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, voltage);
  cellChargedVoltage = voltage;
}
float getCellChargedVoltageEEPROM() {
  float voltage = 0.00f;
  EEPROM.get(CELL_CHARGED_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

void setCellNominalVoltageEEPROM(float voltage) {
  EEPROM.put(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, voltage);
  cellNominalVoltage = voltage;
}
float getCellNominalVoltageEEPROM() {
  float voltage = 0.00f;
  EEPROM.get(CELL_NOMINAL_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

void setCellCriticalVoltageEEPROM(float voltage) {
  EEPROM.put(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, voltage);
  cellCriticalVoltage = voltage;
}
float getCellCriticalVoltageEEPROM() {
  float voltage = 0.00f;
  EEPROM.get(CELL_CRITICAL_VOLTAGE_EEPROM_ADDR, voltage);
  return voltage;
}

void setTempResolutionEEPROM(uint8_t resolution) {
  EEPROM.put(TEMP_RESOLUTION_EEPROM_ADDR, resolution);
  tempResolution = resolution;
  tempSensor.setResolution(resolution);
}
uint8_t getTempResolutionEEPROM() {
  uint8_t resolution = 0;
  EEPROM.get(TEMP_RESOLUTION_EEPROM_ADDR, resolution);
  return resolution;
}

void setTempOverheatCriticalEEPROM(float threshold) {
  EEPROM.put(TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  tempOverheatCriticalThreshold = threshold;
}
float getTempOverheatCriticalEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}

void setTempOverheatWarningEEPROM(float threshold) {
  EEPROM.put(TEMP_OVERHEAT_WARNING_EEPROM_ADDR, threshold);
  tempOverheatWarningThreshold = threshold;
}
float getTempOverheatWarningEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(TEMP_OVERHEAT_WARNING_EEPROM_ADDR, threshold);
  return threshold;
}

void setTempUnderheatCriticalEEPROM(float threshold) {
  EEPROM.put(TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  tempUnderheatCriticalThreshold = threshold;
}
float getTempUnderheatCriticalEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}

void setTempUnderheatWarningEEPROM(float threshold) {
  EEPROM.put(TEMP_UNDERHEAT_WARNING_EEPROM_ADDR, threshold);
  tempUnderheatWarningThreshold = threshold;
}
float getTempUnderheatWarningEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(TEMP_UNDERHEAT_WARNING_EEPROM_ADDR, threshold);
  return threshold;
}
