// Speed & heading control demo
// Runs a fixed sequence: forward → backward → turns → U-turn
// Encoder counts printed so we can verify closed-loop readiness

#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
volatile long encL = 0, encR = 0;

void isr_LA() { if (digitalRead(ENC_LA) == digitalRead(ENC_LB)) encL++; else encL--; }
void isr_LB() { if (digitalRead(ENC_LA) != digitalRead(ENC_LB)) encL++; else encL--; }
void isr_RA() { if (digitalRead(ENC_RA) == digitalRead(ENC_RB)) encR++; else encR--; }
void isr_RB() { if (digitalRead(ENC_RA) != digitalRead(ENC_RB)) encR++; else encR--; }

void setup() {
  Serial.begin(9600);

  pinMode(ENC_LA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LA), isr_LA, RISING);
  pinMode(ENC_LB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LB), isr_LB, RISING);
  pinMode(ENC_RA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RA), isr_RA, RISING);
  pinMode(ENC_RB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RB), isr_RB, RISING);

  Wire1.begin();
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.clearMotorFaultUnconditional();
  mc.setMaxAcceleration(1, 300);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 300);
  mc.setMaxDeceleration(2, 300);

  Serial.println("Speed & heading test starting in 2s...");
  delay(2000);
}

void loop() {
  // --- Forward at speed 400 ---
  Serial.println(">>> Forward 400");
  encL = 0; encR = 0;
  mc.setSpeed(1, 400);
  mc.setSpeed(2, -400);
  delay(2000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  // --- Forward at speed 800 ---
  Serial.println(">>> Forward 800");
  encL = 0; encR = 0;
  mc.setSpeed(1, 800);
  mc.setSpeed(2, -800);
  delay(2000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  // --- Backward at speed 600 ---
  Serial.println(">>> Backward 600");
  encL = 0; encR = 0;
  mc.setSpeed(1, -600);
  mc.setSpeed(2, 600);
  delay(2000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  // --- Turn left 90° (tracks opposite, ~1s at speed 500) ---
  // TODO: calculate exact encoder ticks once TRACK_WIDTH and TICKS_PER_M are known
  Serial.println(">>> Turn left 90");
  encL = 0; encR = 0;
  mc.setSpeed(1, -500);
  mc.setSpeed(2, -500);
  delay(1000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  // --- Turn right 90° ---
  Serial.println(">>> Turn right 90");
  encL = 0; encR = 0;
  mc.setSpeed(1, 500);
  mc.setSpeed(2, 500);
  delay(1000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  // --- U-turn (tracks opposite, ~2s at speed 500) ---
  Serial.println(">>> U-turn (180)");
  encL = 0; encR = 0;
  mc.setSpeed(1, -500);
  mc.setSpeed(2, -500);
  delay(2000);
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  printEncoders();
  delay(1000);

  Serial.println("=== Sequence done, restarting ===");
  delay(3000);
}

void printEncoders() {
  Serial.print("  Encoders L:");
  Serial.print(encL);
  Serial.print(" R:");
  Serial.println(encR);
}
