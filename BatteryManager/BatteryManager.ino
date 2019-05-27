#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define RATE 1

#define WIRE_ID 8

//TODO move num cells to EEPROM
//TODO autodetect number of cells
#define NUM_CELLS 2

#define LED_ONE_WIRE_PIN 6

#define TEMP_ONE_WIRE_PIN 2

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11
#define TEMP_RESOLUTION_EEPROM_ADDR 12 //12
#define TEMP_OVERHEAT_WARNING_EEPROM_ADDR 13 //13-16
#define TEMP_OVERHEAT_CRITICAL_EEPROM_ADDR 17 //17-20
#define TEMP_UNDERHEAT_WARNING_EEPROM_ADDR 21 //21-24
#define TEMP_UNDERHEAT_CRITICAL_EEPROM_ADDR 25 //25-28
#define CURRENT_WARNING_EERPOM_ADDR 29 //29-32
#define CURRENT_CRITICAL_EEPROM_ADDR 33 //33-36

Adafruit_NeoPixel statusLED(1, LED_ONE_WIRE_PIN, NEO_GRB);
//TODO move led brightness to EEPROM
#define STATUS_LED_BRIGHTNESS 70

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

#define CELL_0_VOLTAGE_REGISTER       0x10
#define CELL_1_VOLTAGE_REGISTER       0x20
#define CELL_2_VOLTAGE_REGISTER       0x30
#define CELL_3_VOLTAGE_REGISTER       0x40
#define CELL_4_VOLTAGE_REGISTER       0x50
#define CELL_5_VOLTAGE_REGISTER       0x60
#define CELL_6_VOLTAGE_REGISTER       0x70
#define CELL_7_VOLTAGE_REGISTER       0x80
#define BATTERY_VOLTAGE_REGISTER      0x90
#define BATTERY_CURRENT_REGISTER      0xA0
#define BATTERY_POWER_REGISTER        0xB0
#define BATTERY_TEMPERATURE_REGISTER  0xC0
#define BATTERY_ERROR_REGISTER        0xD0
#define BATTERY_STATUS_REGISTER       0xE0

Adafruit_INA219 ina219;

struct Battery {
  float cellVoltages[NUM_CELLS];
  float voltage;
  float current;
  float power;
  float temperature;
  uint16_t errors;
  byte status;
};

#define ERROR_CELL_OVERVOLTAGE      0x0001
#define ERROR_CELL_LOW              0x0002
#define ERROR_CELL_CRITICAL         0x0004
#define ERROR_OVER_TEMP_WARNING     0x0008
#define ERROR_OVER_TEMP_CRITICAL    0x0010
#define ERROR_UNDER_TEMP_WARNING    0x0020
#define ERROR_UNDER_TEMP_CRITICAL   0x0040
#define ERROR_CURRENT_WARNING       0x0080
#define ERROR_CURRENT_CIRITCAL      0x0100
//IDEA battery disconected error

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

byte wireRegister = CELL_0_VOLTAGE_REGISTER;

void setup() {
  Serial.begin(115200);

  Wire.begin(WIRE_ID);
  Wire.onRequest(wireRequest);
  Wire.onReceive(wireReceive);

  tempResolution = getTempResolutionEEPROM();
  tempOverheatCriticalThreshold = getTempOverheatCriticalEEPROM();
  tempUnderheatCriticalThreshold = getTempUnderheatCriticalEEPROM();
  tempOverheatWarningThreshold = getTempOverheatWarningEEPROM();
  tempUnderheatWarningThreshold = getTempUnderheatWarningEEPROM();

  currentWarningThreshold = getCurrentWarningEEPROM();
  currentCriticalThreshold = getCurrentCriticalEEPROM();

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

  Serial.println("Settings loaded from EEPROM:");
  Serial.print("Cell Charged (V): "); Serial.println(cellChargedVoltage);
  Serial.print("Cell Nominal (V): "); Serial.println(cellNominalVoltage);
  Serial.print("Cell Critical (V): "); Serial.println(cellCriticalVoltage);
  Serial.print("Temp Resolution: "); Serial.println(tempResolution);
  Serial.print("Temp Overheat Warn (C): "); Serial.println(tempOverheatWarningThreshold);
  Serial.print("Temp Overheat Crit (C): "); Serial.println(tempOverheatCriticalThreshold);
  Serial.print("Temp Underheat Warn (C): "); Serial.println(tempUnderheatWarningThreshold);
  Serial.print("Temp Underheat Crit (C): "); Serial.println(tempUnderheatCriticalThreshold);
  Serial.print("Current Warn (mA): "); Serial.println(currentWarningThreshold);
  Serial.print("Current Crit (mA): "); Serial.println(currentCriticalThreshold);
  Serial.println();

  delay(2000);
}

Battery battery;
unsigned long prevPollTime = millis();
void loop() {
  //Update temperature sensors
  if (tempSensor.isConversionComplete())
    tempSensor.requestTemperatures();

  if ((millis() - prevPollTime) > (1000 / RATE)) {
    prevPollTime = millis();

    battery = updateBattery();

//    for (int i = 0; i < NUM_CELLS; i++) {
//      Serial.print("Cell "); Serial.print(i); Serial.print(" Voltage: " );
//      Serial.println(battery.cellVoltages[i], 4);
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
//    Serial.print("Errors: "); Serial.println(battery.errors, BIN);
//    Serial.print("Status: ");
//    Serial.println(battery.status);
//    Serial.println();
  }

}

byte floatBuffer[sizeof(float)];
void wireRequest() {
  Serial.print("Got request for register: ");
  Serial.println(wireRegister, HEX);

  if (wireRegister == CELL_0_VOLTAGE_REGISTER) {
    Serial.println("Responding with cell 0 voltage");
    float cell0Voltage = battery.cellVoltages[0];
    memcpy(floatBuffer, &cell0Voltage, sizeof(cell0Voltage));
    Wire.write(floatBuffer, sizeof(floatBuffer));
  }
}

void wireReceive(int numBytes) {
  Serial.print("Wire receiving: "); Serial.print(numBytes); Serial.println(" bytes");
  if (numBytes == 1) {
    Serial.println("Setting wire register");
    wireRegister = Wire.read();
    Serial.print("Wire register set to: "); Serial.println(wireRegister, HEX);
  }
  else if (numBytes == 5) {
    wireRegister = Wire.read();
    Serial.print("Wire register set to: "); Serial.println(wireRegister, HEX);

    for (int i = 0; i < numBytes - 1; i++) {
      floatBuffer[i] = Wire.read();
    }

    float val;
    memcpy(&val, floatBuffer, sizeof(val));
    Serial.print("Val received: "); Serial.println(val, 4);

    switch (wireRegister) {
      case CELL_CHARGED_VOLTAGE_REGISTER:
        Serial.print("Cell charged voltage is currently: "); Serial.println(getCellChargedVoltageEEPROM());
        Serial.print("Setting cell charged voltage to: "); Serial.println(val);
        setCellChargedVoltageEEPROM(val);
        Serial.print("Cell charged voltage set to: "); Serial.println(getCellChargedVoltageEEPROM());
        break;
      case CELL_NOMINAL_VOLTAGE_REGISTER:
        Serial.print("Cell nominal voltage is currently: "); Serial.println(getCellNominalVoltageEEPROM());
        Serial.print("Setting cell nominal voltage to: "); Serial.println(val);
        setCellNominalVoltageEEPROM(val);
        Serial.print("Cell nominal voltage set to: "); Serial.println(getCellNominalVoltageEEPROM());
        break;
      case CELL_CRITICAL_VOLTAGE_REGISTER:
        Serial.print("Cell critical voltage is currently: "); Serial.println(getCellCriticalVoltageEEPROM());
        Serial.print("Setting cell critical voltage to: "); Serial.println(val);
        setCellCriticalVoltageEEPROM(val);
        Serial.print("Cell critical voltage set to: "); Serial.println(getCellCriticalVoltageEEPROM());
        break;
      default:
        break;
    }
  }
  Serial.println("DONE!");
}

Battery updateBattery() {
  Battery battery;

  //BUG when battery is disconnected cell 0 voltage is approx 0v but cell 1 voltge is ~18v, this causes both cell over and under voltage errors
  for (int i = 0; i < NUM_CELLS; i++) {
    battery.cellVoltages[i] = GetCellVoltage(i);
  }

  battery.voltage = GetBankVoltageAtIndex(NUM_CELLS - 1);
  //another option: use ina219 to calcuate load voltage
  //battery.voltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000);

  battery.current = ina219.getCurrent_mA();
  battery.power = ina219.getPower_mW();
  battery.temperature = tempSensor.getTempCByIndex(0);

  battery.errors = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.cellVoltages[i] >= cellChargedVoltage) {
      battery.errors |= ERROR_CELL_OVERVOLTAGE;
    }

    if (battery.cellVoltages[i] <= cellCriticalVoltage) {
      battery.errors |= ERROR_CELL_CRITICAL;
    }
    else if (battery.cellVoltages[i] <= cellNominalVoltage) {
      battery.errors |= ERROR_CELL_LOW;
    }
  }


  if (battery.temperature <= tempUnderheatCriticalThreshold) {
    battery.errors |= ERROR_UNDER_TEMP_CRITICAL;
  }
  else if (battery.temperature  <= tempUnderheatWarningThreshold) {
    battery.errors |= ERROR_UNDER_TEMP_WARNING;
  }


  if (battery.temperature >= tempOverheatCriticalThreshold) {
    battery.errors |= ERROR_OVER_TEMP_CRITICAL;
  }
  else if (battery.temperature >= tempOverheatWarningThreshold) {
    battery.errors |= ERROR_OVER_TEMP_WARNING;
  }


  if (battery.current >= currentCriticalThreshold) {
    battery.errors |= ERROR_CURRENT_CIRITCAL;
  }
  else if (battery.current >= currentWarningThreshold) {
    battery.errors |= ERROR_CURRENT_WARNING;
  }


  battery.status = 0;
  if (
    battery.errors & ERROR_CELL_OVERVOLTAGE ||
    battery.errors & ERROR_CELL_LOW ||
    battery.errors & ERROR_UNDER_TEMP_WARNING ||
    battery.errors & ERROR_OVER_TEMP_WARNING ||
    battery.errors & ERROR_CURRENT_WARNING
  ) {
    battery.status = 1;
  }

  if (
    battery.errors & ERROR_CELL_CRITICAL ||
    battery.errors & ERROR_UNDER_TEMP_CRITICAL ||
    battery.errors & ERROR_OVER_TEMP_CRITICAL ||
    battery.errors & ERROR_CURRENT_CIRITCAL
  ) {
    battery.status = 2;
  }


  if (battery.status == 0) {
    statusLED.fill(statusLED.Color(0, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == 1) {
    statusLED.fill(statusLED.Color(STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS, 0));
  }
  else if (battery.status == 2) {
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

void setCurrentWarningEEPROM(float threshold) {
  EEPROM.put(CURRENT_WARNING_EERPOM_ADDR, threshold);
  currentWarningThreshold = threshold;
}
float getCurrentWarningEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(CURRENT_WARNING_EERPOM_ADDR, threshold);
  return threshold;
}

void setCurrentCriticalEEPROM(float threshold) {
  EEPROM.put(CURRENT_CRITICAL_EEPROM_ADDR, threshold);
  currentCriticalThreshold = threshold;
}
float getCurrentCriticalEEPROM() {
  float threshold = 0.0f;
  EEPROM.get(CURRENT_CRITICAL_EEPROM_ADDR, threshold);
  return threshold;
}
