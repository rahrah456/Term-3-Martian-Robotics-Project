// mqtt_manager_test.ino — isolate which subsystem crashes MiniMessenger
//
// Step 1: MQTTManager wrapper only                       ✓ no crash
// Step 2: + encoder interrupts (4 ISRs on pins 2-5)      ✗ CRASH
// Step 2b: + encoder POLLING (no ISRs, 50Hz quadrature)  ← you are here
// Step 3: + IMU init + readIMU()
// Step 4: + IR init + readIR()
// Step 5: + UDS readUDS() calls

#include <MiniMessenger.h>
#include <Wire.h>
#include <Motoron.h>

#include "secrets.h"
#include "MQTTManager.h"

MotoronI2C mc(0x10);
MQTTManager mqtt("Terminator");

// ── Encoder polling (50 Hz quadrature — no ISRs) ───────────
volatile long encL = 0, encR = 0;

static const int8_t QUAD_TABLE[16] = {
  0,  1, -1,  0,
 -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

void pollEncoders() {
  static int prevL = 0, prevR = 0;
  int stateL = (digitalRead(PIN_ENC_LA) << 1) | digitalRead(PIN_ENC_LB);
  int stateR = (digitalRead(PIN_ENC_RA) << 1) | digitalRead(PIN_ENC_RB);
  encL += QUAD_TABLE[(prevL << 2) | stateL];
  encR += QUAD_TABLE[(prevR << 2) | stateR];
  prevL = stateL;
  prevR = stateR;
}

// ── Kill switch ISR (only one interrupt — works fine) ──────
volatile bool robotPhysicalEnabled = false;
volatile unsigned long lastDeathButtonTime = 0;
const unsigned long deathDebounceDelay = 500;

void Death() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastDeathButtonTime > deathDebounceDelay) {
    robotPhysicalEnabled = !robotPhysicalEnabled;
  }
  lastDeathButtonTime = interruptTime;
}

bool robotWiFiEnabled = false;
unsigned long lastRegisterMs = 0;

bool isRobotEnabled() {
  return robotWiFiEnabled;
}

unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

void onMqttEnable() {
  robotWiFiEnabled = true;
}

void onMqttDisable() {
  robotWiFiEnabled = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ── Encoder pins (polled, no ISRs) ───────────────────────
  pinMode(PIN_ENC_LA, INPUT_PULLUP);
  pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP);
  pinMode(PIN_ENC_RB, INPUT_PULLUP);

  pinMode(PIN_ACT_LED, OUTPUT);
  digitalWrite(PIN_ACT_LED, HIGH);

  mqtt.onEnable = onMqttEnable;
  mqtt.onDisable = onMqttDisable;
  mqtt.begin();

  Serial.println("MQTT init done");

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

  pinMode(PIN_KILL_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_KILL_BTN), Death, FALLING);

  Serial.println("Setup done");
}

void loop() {
  mqtt.loop();
  pollEncoders();  // called every loop, ~50Hz in final code

  // Print encoder counts every 5s to verify polling works
  static unsigned long lastEncPrint = 0;
  if (millis() - lastEncPrint > 5000) {
    lastEncPrint = millis();
    Serial.print("enc: ");
    Serial.print(encL);
    Serial.print(" ");
    Serial.println(encR);
  }

  if (robotWiFiEnabled && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    robotWiFiEnabled = false;
    Serial.println("Heartbeat timeout");
  }

  if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    char reg[64];
    snprintf(reg, sizeof(reg),
             "type=register team_id=%s board_id=%s",
             GROUP_ID, "Terminator");
    mqtt.messenger.sendToBoard("server", reg);
    Serial.println("Registered with server");
  }

  if (!robotPhysicalEnabled) {
    digitalWrite(PIN_ACT_LED, HIGH);
    mc.setSpeed(1, 0);
    mc.setSpeed(2, 0);
    return;
  } else {
    if (isRobotEnabled()) {
      digitalWrite(PIN_ACT_LED, LOW);
    } else {
      digitalWrite(PIN_ACT_LED, HIGH);
      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);
      return;
    }
  }

  mc.setSpeed(1, 660);
  mc.setSpeed(2, -660);
}
