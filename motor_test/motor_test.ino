#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

// --- Encoder Settings (Motor 2 only) ---
const int ENCODER_PIN_1A = 2;
const int ENCODER_PIN_1B = 3;
const int ENCODER_PIN_2A = 4;
const int ENCODER_PIN_2B = 5;

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// --- Encoder ISRs ---
void encoderISR_1A() {
  if (digitalRead(ENCODER_PIN_1A) == digitalRead(ENCODER_PIN_1B)) {
    encoderCountLeft++;
  }
  else {
    encoderCountLeft--;
  }
}

void encoderISR_1B() {
  if (digitalRead(ENCODER_PIN_1A) != digitalRead(ENCODER_PIN_1B)) {
    encoderCountLeft++;
  }
  else {
    encoderCountLeft--;
  }
}

void encoderISR_2A() {
  if (digitalRead(ENCODER_PIN_2A) == digitalRead(ENCODER_PIN_2B)) {
    encoderCountRight++;
  }
  else {
    encoderCountRight--;
  }
}

void encoderISR_2B() {
  if (digitalRead(ENCODER_PIN_2A) != digitalRead(ENCODER_PIN_2B)) {
    encoderCountRight++;
  }
  else {
    encoderCountRight--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN_1A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_1B, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1A), encoderISR_1A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1B), encoderISR_1B, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2A), encoderISR_2A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2B), encoderISR_2B, RISING);

  Wire1.begin();
  mc.setBus(&Wire1);

  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.clearMotorFaultUnconditional();

  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 200);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 200);
}

void loop() {
  // 1. Spin motors FORWARD at maximum speed
  mc.setSpeed(1, 800);
  mc.setSpeed(2, -800);
  delay(2000); 

  // 2. Stop motors
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  delay(1000); 

  Serial.print(encoderCountLeft);
  Serial.print(" ");
  Serial.println(encoderCountRight);

  // 3. Spin motors BACKWARD at maximum speed
  mc.setSpeed(1, -800);
  mc.setSpeed(2, -800);
  delay(2000); 

  // 4. Stop motors
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  delay(1000);

  Serial.print(encoderCountLeft);
  Serial.print(" ");
  Serial.println(encoderCountRight);
}