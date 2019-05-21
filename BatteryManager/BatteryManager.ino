#include <Wire.h>
#define WIRE_ID 8

#define nominalCuttoffVoltage 3.7
#define depletedCutoffVoltage 3.0

#define RedLED 2
#define YellowLED 3
#define GreenLED 4

#define cell1Pin A0

#define cell2Pin A1
#define cell2Resistor1 0.979
#define cell2Resistor2 1.152

#define cell3Pin A2
#define cell3Resistor1 2
#define cell3Resistor2 1.127

#define currentPin A3

struct Battery {
  float cellVoltages[8];
  float batteryVoltage;
  float current;
  byte status;
};

void setup() {
  Serial.begin(115200);
  Wire.begin(WIRE_ID);
  Wire.onRequest(wireRequest);

  analogReference(DEFAULT);

  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  digitalWrite(RedLED, LOW);
  digitalWrite(YellowLED, LOW);
  digitalWrite(GreenLED, LOW);
}

void loop() {

  Battery battery = updateBattery();

  for (int i = 0; i < 8; i++) {
    Serial.print("Cell ");Serial.print(i);Serial.print(" Voltage: " );
    Serial.println(battery.cellVoltages[i]);
  }

  Serial.print("Battery Voltage: ");
  Serial.println(battery.batteryVoltage);
  Serial.print("\tCurrent (mA): ");
  Serial.println(battery.current);
  Serial.print("\tStatus: ");
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

  battery.cellVoltages[0] = Voltage(cell1Pin);
  battery.cellVoltages[1] = Voltage(cell2Pin, cell2Resistor1, cell2Resistor2) - battery.cellVoltages[0];
  battery.cellVoltages[2] = Voltage(cell3Pin, cell3Resistor1, cell3Resistor2) - battery.cellVoltages[1];
  //  battery.battery = Voltage(cell3Pin, cell3Resistor1, cell3Resistor2);
  battery.batteryVoltage = Voltage(cell2Pin, cell2Resistor1, cell2Resistor2);

  battery.current = mapf(analogRead(currentPin), 0, 1023, 20000, -20000);

  if ( (battery.cellVoltages[0] <= depletedCutoffVoltage) || (battery.cellVoltages[1] <= depletedCutoffVoltage) || (battery.cellVoltages[2] <= depletedCutoffVoltage) ) {
    digitalWrite(RedLED, HIGH);
    digitalWrite(YellowLED, LOW);
    digitalWrite(GreenLED, LOW);
    battery.status = 2;
  }
  else if ( (battery.cellVoltages[0] <= nominalCuttoffVoltage) || (battery.cellVoltages[1] <= nominalCuttoffVoltage) || (battery.cellVoltages[2] <= nominalCuttoffVoltage) ) {
    digitalWrite(RedLED, LOW);
    digitalWrite(YellowLED, HIGH);
    digitalWrite(GreenLED, LOW);
    battery.status = 1;
  }
  else {
    digitalWrite(RedLED, LOW);
    digitalWrite(YellowLED, LOW);
    digitalWrite(GreenLED, HIGH);
    battery.status = 0;
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
