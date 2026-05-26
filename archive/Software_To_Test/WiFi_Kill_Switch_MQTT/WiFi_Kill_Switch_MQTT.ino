#include <MiniMessenger.h>
#include "secrets.h"

// ── Configuration ────────────────────────────────────────────────────────────
MiniMessenger messenger;
const char* BoardId = "GIGA_1";   // Change to your robot's name

// ── Kill Switch State ─────────────────────────────────────────────────────────
volatile bool robotEnabled = false;   // Start DISABLED for safety

// ── Public helper – call this anywhere in your code ──────────────────────────
bool isRobotEnabled() {
  return robotEnabled;
}

// ── MQTT Message Handler ──────────────────────────────────────────────────────
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {

  // Build a null-terminated string from the payload
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("[MQTT] From ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(msg);

  // ── Kill switch commands ──────────────────────────────────────────────────
  if (strcmp(msg, "ENABLE") == 0) {
    robotEnabled = true;
    Serial.println(">>> ROBOT ENABLED <<<");
    onRobotEnabled();

  } else if (strcmp(msg, "DISABLE") == 0) {
    robotEnabled = false;
    Serial.println(">>> ROBOT DISABLED <<<");
    onRobotDisabled();
  }
}

// ── Actions on state change ───────────────────────────────────────────────────
void onRobotEnabled() {
  // TODO: release brakes, signal LEDs, etc.
}

void onRobotDisabled() {
  // TODO: stop motors immediately, engage brakes, etc.
  // Example: motor.stop();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  Serial.println("Kill switch ready – waiting for ENABLE command.");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  messenger.loop();   // MUST be called every loop iteration

  if (!isRobotEnabled()) {
    // Hard block – nothing below runs while disabled
    return;
  }

  // ── Your robot logic goes here ────────────────────────────────────────────
  // All of this is skipped if the kill switch is inactive
}