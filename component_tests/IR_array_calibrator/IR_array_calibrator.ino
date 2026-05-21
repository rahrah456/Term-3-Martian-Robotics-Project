const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {30, 23, 24, 25, 26, 27, 28, 29, 22};
const int IR_EMITTER_1 = 40;
const int IR_EMITTER_2 = 41;
const int TIMEOUT = 2500;

uint16_t calMins[IR_COUNT];
uint16_t calMaxs[IR_COUNT];

void readRawIR(uint16_t* rawVals) {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
    rawVals[i] = TIMEOUT; 
  }
  delayMicroseconds(10); 
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  unsigned long startTime = micros();
  uint8_t pinsLeft = IR_COUNT;
  while (pinsLeft > 0) {
    unsigned long elapsed = micros() - startTime;
    if (elapsed >= TIMEOUT) break;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      if (rawVals[i] == TIMEOUT && digitalRead(IR_PINS[i]) == LOW) {
        rawVals[i] = elapsed;
        pinsLeft--;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  pinMode(IR_EMITTER_1, OUTPUT);
  pinMode(IR_EMITTER_2, OUTPUT);
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);

  // Initialize calibration tracking variables
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    calMins[i] = TIMEOUT;
    calMaxs[i] = 0;
  }

  Serial.println("Starting calibration in 3 seconds...");
  Serial.println("Get ready to sweep the sensors across the line!");
  delay(3000);
  Serial.println("CALIBRATING... Move the robot back and forth NOW!");

  uint16_t rawOff[IR_COUNT];
  uint16_t rawOn[IR_COUNT];

  // Run calibration for ~5 seconds
  for (uint16_t iter = 0; iter < 250; iter++) {
    readRawIR(rawOff);
    digitalWrite(IR_EMITTER_1, HIGH);
    digitalWrite(IR_EMITTER_2, HIGH);
    delayMicroseconds(200); 
    readRawIR(rawOn);
    digitalWrite(IR_EMITTER_1, LOW);
    digitalWrite(IR_EMITTER_2, LOW);

    for (uint8_t i = 0; i < IR_COUNT; i++) {
      int32_t adjustedRaw = (int32_t)rawOn[i] + TIMEOUT - (int32_t)rawOff[i];
      if (adjustedRaw > TIMEOUT) adjustedRaw = TIMEOUT;
      if (adjustedRaw < 0) adjustedRaw = 0;

      // Track the highest and lowest values seen for each pin
      if (adjustedRaw < calMins[i]) calMins[i] = adjustedRaw;
      if (adjustedRaw > calMaxs[i]) calMaxs[i] = adjustedRaw;
    }
    delay(10);
  }

  Serial.println("\nCalibration complete!\n");
  Serial.println("==================================================");
  Serial.println("COPY THESE TWO LINES INTO YOUR MAIN CODE:");
  Serial.println("==================================================\n");

  Serial.print("uint16_t myMins[9] = {");
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    Serial.print(calMins[i]);
    if (i < IR_COUNT - 1) Serial.print(", ");
  }
  Serial.println("};");

  Serial.print("uint16_t myMaxs[9] = {");
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    Serial.print(calMaxs[i]);
    if (i < IR_COUNT - 1) Serial.print(", ");
  }
  Serial.println("};\n");
}

void loop() {
  // Do nothing, just wait
}
