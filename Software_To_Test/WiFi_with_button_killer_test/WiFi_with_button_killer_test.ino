#include <MiniMessenger.h>
#include "secrets.h"

#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

const int killSwitch = 38;

volatile bool robotPhysicalEnabled = false;
volatile unsigned long lastDeathButtonTime = 0;
const unsigned long deathDebounceDelay = 500; // ms


// ── Configuration ────────────────────────────────────────────────────────────
MiniMessenger messenger;
const char* BoardId = "wippy";   // Change to your robot's name
const int LED_pin = 39;
unsigned long lastRegisterMs = 0;
unsigned long lastMsgMs = 0;
bool robotWiFiEnabled = false;      // Controlled by Heartbeats/Emergency commands

// ── Public helper – call this anywhere in your code ──────────────────────────
bool isRobotEnabled() {
  return robotWiFiEnabled;
}

// --- SAFETY WATCHDOG ---
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000; // 4x the 250ms server interval

// This function runs every time an MQTT message arrives for this robot
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[128];
  size_t copyLen = (length < 127) ? length : 127;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.println(msg);

  // 1. Heartbeat Check
  if (strstr(msg, "type=heartbeat")) {
      lastHeartbeatMs = millis(); // Track that server is alive

      if (strstr(msg, "enable=1")) {
          robotWiFiEnabled = true;
      } else if (strstr(msg, "enable=0")) {
          robotWiFiEnabled = false;
          Serial.println("SAFETY: Heartbeat Disabled");
      }
  }

  // 2. Emergency/Disable Check (Immediate Stop)
  if (strstr(msg, "type=emergency enabled=true") || 
      strstr(msg, "type=disable enabled=false")) {
      robotWiFiEnabled = false;
      Serial.println("!!! EMERGENCY/DISABLE ACTIVE !!!");
  }
}

// ── Actions on state change ───────────────────────────────────────────────────
void onRobotEnabled() {
  // TODO: release brakes, signal LEDs, etc.
  digitalWrite(LED_pin, LOW); //Sets flashing LED
}

void onRobotDisabled() {
  // TODO: stop motors immediately, engage brakes, etc.
  // Example: motor.stop();
  digitalWrite(LED_pin, HIGH); //Sets flashing LED
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  delay(1000);

  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, HIGH); //Sets flashing LED

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  Serial.println("Kill switch ready - waiting for ENABLE command.");

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

  pinMode(killSwitch, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(killSwitch),
    Death,
    FALLING
  );
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  messenger.loop();   // MUST be called every loop iteration

  // --- HEARTBEAT WATCHDOG ---
  // If we were enabled but haven't heard from the server in a while, disable.
  if (robotWiFiEnabled && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
      robotWiFiEnabled = false;
      Serial.println("SAFETY: Heartbeat Timeout (Server Connection Lost)");
  }

  //REGISTRATION: Tell the server we are here every 10 seconds
  if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
      lastRegisterMs = millis();
      char reg[64];
      snprintf(reg,
                sizeof(reg), 
                "type=register team_id=%s board_id=%s",
                GROUP_ID, 
                BoardId);
      messenger.sendToBoard("server", reg);
      Serial.println("Registered with server.");
  }

  //nested conditions to make physical higher level than wifi
  if (!robotPhysicalEnabled) {
    onRobotDisabled();

    mc.setSpeed(1, 0);
    mc.setSpeed(2, 0);
    return;
  }
  else{
    if(isRobotEnabled()){
      onRobotEnabled();
    }
    else{
      onRobotDisabled();

      mc.setSpeed(1, 0);
      mc.setSpeed(2, 0);

      // Hard block – nothing below runs while disabled
      return; // can be removed, replace by adding motors set to 0 accel/deccel so sensing can happen
    }
  }

  

  // ── Your robot logic goes here ────────────────────────────────────────────
  // All of this is skipped if the kill switch is inactive

  /*
  // Send msg test ------------------------------------------------------------
  if (millis() - lastMsgMs > 10000 || lastMsgMs == 0) {
    lastMsgMs = millis();

    char isF[64];
    snprintf(isF,
              sizeof(isF), 
              "type=seedPlanted tag_id=23C9AA41 board_id=%s", 
              BoardId);
    messenger.sendToBoard("server", isF);
  }
  */

  
  mc.setSpeed(1, 660);
  mc.setSpeed(2, -660);
}

void Death() {

  unsigned long interruptTime = millis();

  // Ignore interrupts that happen too quickly
  if (interruptTime - lastDeathButtonTime > deathDebounceDelay) {
    robotPhysicalEnabled = !robotPhysicalEnabled;
  }

  lastDeathButtonTime = interruptTime;
}