#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
volatile long encL = 0, encR = 0;

void isr_LA() { encL += (digitalRead(ENC_LA) == digitalRead(ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(ENC_LA) != digitalRead(ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(ENC_RA) != digitalRead(ENC_RB)) ? 1 : -1; }

void setMotors(int left, int right) {
  mc.setSpeed(1, left);
  mc.setSpeed(2, -right);
}

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
  mc.setMaxAcceleration(1, 800);
  mc.setMaxDeceleration(1, 800);
  mc.setMaxAcceleration(2, 800);
  mc.setMaxDeceleration(2, 800);
  setMotors(0, 0);

  while (!Serial) {
    delay(5);
  }
  Serial.println("t(s)\tpower\tencL\tencR");
}

void loop() {
  // // 0s-3s: idle at 0
  // setMotors(0, 0);
  // for (int i = 0; i < 3; i++) {
  //   Serial.print(millis() / 1000); Serial.print("\t0\t");
  //   Serial.print(encL); Serial.print("\t"); Serial.println(encR);
  //   delay(1000);
  // }

  // // 3s-18s: ramp 600 -> 800 over 15s
  // for (int p = 600; p <= 800; p++) {
  //   setMotors(p, p);
  //   delay(1.5*15000.0 / 800.0);
  //   if (p == 0 || p == 800 || p % 50 == 0) {
  //     Serial.print(millis() / 1000); Serial.print("\t"); Serial.print(p); Serial.print("\t");
  //     Serial.print(encL); Serial.print("\t"); Serial.println(encR);
  //   }
  // }

  // setMotors(600, 600);
  // // 18s-27s: hold at 600
  // for (int i = 0; i < 15; i++) {
  //   Serial.print(millis() / 1000); Serial.print("\t800\t");
  //   Serial.print(encL); Serial.print("\t"); Serial.println(encR);
  //   delay(1000);
  // }
  setMotors(300, 300);
  // 18s-27s: hold at 600
  for (int i = 0; i < 15; i++) {
    Serial.print(millis() / 1000); Serial.print("\t800\t");
    Serial.print(encL); Serial.print("\t"); Serial.println(encR);
    delay(1000);
  }

  setMotors(0, 0);
  Serial.println("Done");
  while (1);
}
