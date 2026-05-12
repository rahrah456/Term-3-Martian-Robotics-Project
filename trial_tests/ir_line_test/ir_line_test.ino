const uint8_t SENSOR_COUNT = 9;
const uint8_t SENSOR_PINS[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};

void setup() {
  Serial.begin(9600);
  Serial.println("IR array test — raw values + weighted centroid");
}

void loop() {
  uint16_t vals[SENSOR_COUNT];
  readIR(vals);

  // print raw
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(vals[i]);
    Serial.print("\t");
  }

  // weighted centroid (0 = far left, 8000 = far right)
  uint32_t sum = 0, weighted = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t v = 1000 - vals[i]; // darker = lower = more reflective
    if (v > 50) {
      sum += v;
      weighted += v * i * 1000;
    }
  }
  if (sum > 0) {
    Serial.print(" | Line pos: ");
    Serial.println(weighted / sum);
  } else {
    Serial.println(" | No line");
  }

  delay(100);
}

void readIR(uint16_t vals[]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], OUTPUT);
    digitalWrite(SENSOR_PINS[i], HIGH);
  }
  delayMicroseconds(10);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    vals[i] = 1000;
  }

  unsigned long start = micros();
  while (micros() - start < 1000) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (vals[i] == 1000 && digitalRead(SENSOR_PINS[i]) == LOW) {
        vals[i] = micros() - start;
      }
    }
  }
}
