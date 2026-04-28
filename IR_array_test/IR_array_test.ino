const uint8_t SensorCount = 9;
const uint8_t sensorPins[] = {2, 3,4,5,6,7,8,9,10};

void setup() {
  Serial.begin(9600);
}

void loop() {
  uint16_t sensorValues[SensorCount];

  Read_IR_Array(sensorValues, SensorCount, sensorPins);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(250);
}

void Read_IR_Array(uint16_t sensorValues[], uint8_t SensorCount, const uint8_t sensorPins[]) {

  // 1. Charge capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // 2. Switch to input
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = 1000;
  }

  // 3. Measure discharge
  unsigned long startTime = micros();
  while (micros() - startTime < 1000) {
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == 1000 && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
      }
    }
  }
}