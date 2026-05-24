#include <Wire.h>
#include <Motoron.h>

// --- Pins ---
const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
const int KILL_BTN = 38;
const int IR_EMITTER_1 = 40;
const int IR_EMITTER_2 = 41;
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {30, 23, 24, 25, 26, 27, 28, 29, 22};
const int TIMEOUT = 2500;

// Calibrated IR min/max from trial1_demo
const uint16_t IR_MINS[9] = { 57, 31, 29, 26, 36, 44, 58, 81, 111 };
const uint16_t IR_MAXS[9] = { 1577, 1173, 1067, 946, 946, 974, 998, 1173, 1501 };

const int MOTOR_MIN = 300;
const int MOTOR_MAX = 660;
int maxDiff = 40;
const int CENTER_TARGET = 4000; // centre of 9 sensors (0-8000)

MotoronI2C mc(0x10);

volatile long encL = 0, encR = 0;
bool killed = false;

void isr_LA() { encL += (digitalRead(ENC_LA) == digitalRead(ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(ENC_LA) != digitalRead(ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(ENC_RA) != digitalRead(ENC_RB)) ? 1 : -1; }

void setMotors(int left, int right) {
  mc.setSpeed(1, left);
  mc.setSpeed(2, -right);
}

bool handleEStop() {
  static bool lastBtn = HIGH;
  static unsigned long debounce = 0;
  bool btn = digitalRead(KILL_BTN);
  if (btn == HIGH && lastBtn == LOW && millis() - debounce > 50) {
    debounce = millis();
    killed = !killed;
    if (killed) { setMotors(0, 0); Serial.println("E-STOP engaged"); }
    else { Serial.println("E-STOP released"); }
  }
  lastBtn = btn;
  return killed;
}

void waitForUnkill() {
  while (killed) { handleEStop(); delay(20); }
}

// --- IR reading (same as trial1_demo) ---
void readRawIR(uint16_t* rawVals) {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
    rawVals[i] = TIMEOUT;
  }
  delayMicroseconds(10);
  for (uint8_t i = 0; i < IR_COUNT; i++) pinMode(IR_PINS[i], INPUT);

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

void readIR(uint16_t vals[]) {
  uint16_t rawOff[IR_COUNT];
  uint16_t rawOn[IR_COUNT];

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
    long mapped = map(adjustedRaw, IR_MINS[i], IR_MAXS[i], 0, 1000);
    vals[i] = constrain(mapped, 0, 1000);
  }
}

int irCentroid(uint16_t vals[]) {
  uint32_t sum = 0, weighted = 0;
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    uint16_t v = 1000 - vals[i];
    if (v > 50) { sum += v; weighted += v * i * 1000; }
  }
  if (sum < 100) return -1;
  return weighted / sum;
}

float kp = 0.5, ki = 0.0, kd = 0.0;

void printParams() {
  Serial.print("kp:"); Serial.print(kp, 3);
  Serial.print(" ki:"); Serial.print(ki, 3);
  Serial.print(" kd:"); Serial.print(kd, 3);
  Serial.print(" md:"); Serial.println(maxDiff);
}

bool handleSerialCommand() {
  if (!Serial.available()) return false;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    while (Serial.available()) Serial.read();
    return false;
  }

  int sep = line.indexOf(':');
  if (sep < 0) {
    // non-command char — stop
    while (Serial.available()) Serial.read();
    return true;
  }

  String key = line.substring(0, sep);
  float val = line.substring(sep + 1).toFloat();

  if (key == "kp") { kp = val; Serial.print("kp = "); Serial.println(kp, 3); }
  else if (key == "ki") { ki = val; Serial.print("ki = "); Serial.println(ki, 3); }
  else if (key == "kd") { kd = val; Serial.print("kd = "); Serial.println(kd, 3); }
  else if (key == "md") { maxDiff = constrain((int)val, 0, 130); Serial.print("maxDiff = "); Serial.println(maxDiff); }
  else { Serial.print("Unknown: "); Serial.println(line); }

  while (Serial.available()) Serial.read();
  return false;
}

// --- Line following ---
void lineFollow(int baseSpeed) {
  unsigned long prevTime = millis();
  unsigned long lastPrint = 0;

  bool wasHole = false;
  unsigned long holeStart = 0;

  // Used to hold motor command during holes
  int holdL = baseSpeed;
  int holdR = baseSpeed;
  float integral = 0, prevErr = 0;

  Serial.println("Line following started");
  printParams();
  Serial.println("Send kp:/ki:/kd:/md:<val> to tune, any other char to stop");

  while (true) {
    if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
    if (handleSerialCommand()) { break; }

    uint16_t vals[IR_COUNT];
    readIR(vals);
    int centroid = irCentroid(vals);

    if (centroid < 0) {
      if (!wasHole) {
        // First no-line reading — assume hole, hold motors
        wasHole = true;
        holeStart = millis();
        delay(20);
        continue;
      }
      if (millis() - holeStart < 200) {
        // Still likely crossing a hole — keep previous command
        setMotors(holdL, holdR);
        delay(20);
        continue;
      }
      // No line for >200ms — lost
      setMotors(0, 0);
      Serial.println("Line lost");
      break;
    }

    wasHole = false;

    // PID
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;
    if (dt <= 0 || dt > 0.5) dt = 0.02;

    float error = (float)centroid - CENTER_TARGET;
    integral += error * dt;
    integral = constrain(integral, -50, 50);
    float deriv = (error - prevErr) / dt;
    prevErr = error;

    float correction = kp * error + ki * integral + kd * deriv;
    correction = constrain(correction, -maxDiff, maxDiff);

    int leftMotor = baseSpeed + (int)correction;
    int rightMotor = baseSpeed - (int)correction;

    leftMotor = constrain(leftMotor, MOTOR_MIN, MOTOR_MAX);
    rightMotor = constrain(rightMotor, MOTOR_MIN, MOTOR_MAX);

    holdL = leftMotor;
    holdR = rightMotor;
    setMotors(leftMotor, rightMotor);

    if (now - lastPrint >= 1000) {
      lastPrint = now;
      Serial.print("cent:"); Serial.print(centroid);
      Serial.print(" err:"); Serial.print(error, 1);
      Serial.print(" corr:"); Serial.print(correction, 1);
      Serial.print(" L:"); Serial.print(leftMotor);
      Serial.print(" R:"); Serial.print(rightMotor);
      Serial.print(" ir:");
      for (uint8_t i = 0; i < IR_COUNT; i++) {
        Serial.print(vals[i]);
        if (i < IR_COUNT - 1) Serial.print(",");
      }
      Serial.println();
    }

    delay(20);
  }
  setMotors(0, 0);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(1500);

  pinMode(ENC_LA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LA), isr_LA, RISING);
  pinMode(ENC_LB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LB), isr_LB, RISING);
  pinMode(ENC_RA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RA), isr_RA, RISING);
  pinMode(ENC_RB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RB), isr_RB, RISING);

  pinMode(KILL_BTN, INPUT_PULLUP);
  pinMode(IR_EMITTER_1, OUTPUT);
  pinMode(IR_EMITTER_2, OUTPUT);
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);

  Wire1.begin();
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.clearMotorFaultUnconditional();
  mc.setMaxAcceleration(1, 800);
  mc.setMaxDeceleration(1, 800);
  mc.setMaxAcceleration(2, 800);
  mc.setMaxDeceleration(2, 800);
  setMotors(0, 0);

  Serial.println("Line Following ready");
  Serial.print("Enter base speed (300-660): ");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (Serial.available()) {
    int speed = Serial.parseInt();
    while (Serial.available()) Serial.read();
    if (speed >= MOTOR_MIN && speed <= MOTOR_MAX) {
      lineFollow(speed);
    }
    Serial.print("Enter base speed (300-660): ");
  }
  delay(20);
}
