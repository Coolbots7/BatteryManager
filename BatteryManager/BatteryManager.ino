#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define WIRE_ID 8

#define NUM_CELLS 2

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11

#define redLED 2
#define yellowLED 3
#define greenLED 4

#define ADC_GAIN GAIN_TWOTHIRDS
#define ADC_0_ADDR 0x48
#define ADC_1_ADDR 0x49
Adafruit_ADS1115 ads0(ADC_0_ADDR);
Adafruit_ADS1115 ads1(ADC_1_ADDR);
const float VOLTAGE_DIVIDER_R1[] = {
  0.0f,
  1.761f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
};

const float VOLTAGE_DIVIDER_R2[] = {
  1.0f,
  3.270f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
};

struct Battery {
  float cellVoltages[NUM_CELLS];
  float voltage;
  float current;
  byte status;
};

float cellChargedVoltage;
float cellNominalVoltage;
float cellCriticalVoltage;

void setup() {
  Serial.begin(115200);
  Wire.begin(WIRE_ID);
  Wire.onRequest(wireRequest);

  ads0.begin();
  ads0.setGain(ADC_GAIN);
  
  ads1.begin();
  ads1.setGain(ADC_GAIN);

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  cellChargedVoltage = getCellChargedVoltageEEPROM();
  cellNominalVoltage = getCellNominalVoltageEEPROM();
  cellCriticalVoltage = getCellCriticalVoltageEEPROM();
}

void loop() {

  Battery battery = updateBattery();

  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.print("Cell "); Serial.print(i); Serial.print(" Voltage: " );
    Serial.println(battery.cellVoltages[i]);
  }

  Serial.print("Battery Voltage: ");
  Serial.println(battery.voltage);
  Serial.print("Current (mA): ");
  Serial.println(battery.current);
  Serial.print("Status: ");
  Serial.println(battery.status);
  Serial.println();

  delay(500);

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

  //battery.current = mapf(analogRead(currentPin), 0, 1023, 20000, -20000);

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

  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.cellVoltages[i] <= cellCriticalVoltage) {
      battery.status = 3;
    }
  }

  if (battery.status == 0) {
    digitalWrite(redLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, HIGH);
  }
  else if (battery.status == 2 || battery.status == 1) {
    digitalWrite(redLED, LOW);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  else if (battery.status == 3) {
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, LOW);
  }

  return battery;
}

float Voltage(int analogPin, float resistor1, float resistor2) {
  return Voltage(analogPin) / (resistor2 / (resistor1 + resistor2));
}

float Voltage(int analogPin) {
  return mapf(analogRead(analogPin), 0.0, 1023.0, 0.0, 4.88);
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

float mapf(float in, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (in - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow;
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
