#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

const int KILL_BUTTON = 38;
const int ACT_LED = 39;

const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
volatile long encL = 0, encR = 0;

bool killed = true;
unsigned long lastDebounce = 0;
bool lastRaw = HIGH;

void isr_LA() { if (digitalRead(ENC_LA) == digitalRead(ENC_LB)) encL++; else encL--; }
void isr_LB() { if (digitalRead(ENC_LA) != digitalRead(ENC_LB)) encL++; else encL--; }
void isr_RA() { if (digitalRead(ENC_RA) == digitalRead(ENC_RB)) encR++; else encR--; }
void isr_RB() { if (digitalRead(ENC_RA) != digitalRead(ENC_RB)) encR++; else encR--; }

void setup() {
  Serial.begin(9600);

  pinMode(KILL_BUTTON, INPUT_PULLUP);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, HIGH); // start stopped

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
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 200);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 200);

  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);

  Serial.println("Mechanical kill switch test — press button to toggle");
}

void loop() {
  bool curr = digitalRead(KILL_BUTTON);

  // rising edge with debounce
  if (curr == HIGH && lastRaw == LOW && (millis() - lastDebounce) > 50) {
    lastDebounce = millis();
    killed = !killed;
    if (killed) {
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      digitalWrite(ACT_LED, HIGH);
      Serial.println("STOPPED — LED flashing red");
    } else {
      mc.setSpeed(1, 400);
      mc.setSpeed(2, -400);
      digitalWrite(ACT_LED, LOW);
      Serial.println("RUNNING — moving forward");
    }
  }
  lastRaw = curr;

  if (!killed) {
    Serial.print("Encoders: ");
    Serial.print(encL);
    Serial.print(" ");
    Serial.println(encR);
  }

  delay(50);
}
