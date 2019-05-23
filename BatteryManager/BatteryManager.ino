#include <EEPROM.h>

#include <Wire.h>
#define WIRE_ID 8

#define NUM_CELLS 2

#define CELL_CHARGED_VOLTAGE_EEPROM_ADDR 0 //0-3
#define CELL_NOMINAL_VOLTAGE_EEPROM_ADDR 4 //4-7
#define CELL_CRITICAL_VOLTAGE_EEPROM_ADDR 8 //8-11

#define CELL_CHARGED_VOLTAGE_REGISTER 0x01
#define CELL_NOMINAL_VOLTAGE_REGISTER 0x02
#define CELL_CRITICAL_VOLTAGE_REGISTER 0x03

#define redLED 2
#define yellowLED 3
#define greenLED 4

#define cell1Pin A0

#define cell2Pin A1
#define cell2Resistor1 0.979
#define cell2Resistor2 1.152

#define cell3Pin A2
#define cell3Resistor1 2
#define cell3Resistor2 1.127

#define currentPin A3

struct Battery {
  float cellVoltages[NUM_CELLS];
  float batteryVoltage;
  float current;
  byte status;
};

float cellChargedVoltage;
float cellNominalVoltage;
float cellCriticalVoltage;

uint8_t wireRegister;

void setup() {
  Serial.begin(115200);

  Wire.begin(WIRE_ID);
  Wire.onRequest(wireRequest);
  Wire.onReceive(wireReceive);

  analogReference(DEFAULT);

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
  Serial.println(battery.batteryVoltage);
  Serial.print("Current (mA): ");
  Serial.println(battery.current);
  Serial.print("Status: ");
  Serial.println(battery.status);
  Serial.println();

  delay(500);

}

void wireRequest() {
  //  Serial.println("Sending Battery info");
  //
  //  Battery battery = updateBattery();
  //
  //  byte buffer[sizeof(battery)];
  //  memcpy(buffer, &battery, sizeof(battery));
  //  Wire.write(buffer, sizeof(buffer));


  switch (wireRegister) {
    default:
      break;
  }
}

void wireReceive() {
  if (Wire.available() == 1) {
    wireRegister = Wire.read();
  }
  else if (Wire.available() == 5) {
    wireRegister = Wire.read();

    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    uint8_t b4 = Wire.read();

    float val = (float)(b1 << 24 | b2 << 16 | b3 << 8 | b4);

    switch (wireRegister) {
    case CELL_CHARGED_VOLTAGE_REGISTER:
      setCellChargedVoltageEEPROM(val);
        break;
      default:
        break;
    }

  }
}

Battery updateBattery() {
  Battery battery;

  battery.cellVoltages[0] = Voltage(cell1Pin);
  battery.cellVoltages[1] = Voltage(cell2Pin, cell2Resistor1, cell2Resistor2) - battery.cellVoltages[0];
  battery.cellVoltages[2] = Voltage(cell3Pin, cell3Resistor1, cell3Resistor2) - battery.cellVoltages[1];
  //  battery.battery = Voltage(cell3Pin, cell3Resistor1, cell3Resistor2);
  battery.batteryVoltage = Voltage(cell2Pin, cell2Resistor1, cell2Resistor2);

  battery.current = mapf(analogRead(currentPin), 0, 1023, 20000, -20000);

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
