#ifndef MQTTMANAGER_H
#define MQTTMANAGER_H

#include <Arduino.h>
#include <MiniMessenger.h>
#include "secrets.h"   // Copy from project root into this directory
#include "Config.h"
#include "Map.h"

// ============================================================
//  MQTT MANAGER  —  MiniMessenger wrapper for robot comms
//  Handles ENABLE/DISABLE, PID tuning, hole status, test cmds.
// ============================================================

// Forward declarations for callbacks
class MQTTManager;

// Global pointer so the static callback can reach our instance
static MQTTManager* g_mqtt = nullptr;

class MQTTManager {
public:
  MiniMessenger messenger;
  const char* boardId;
  bool enabled;

  // Callbacks set by main code
  void (*onEnable)();
  void (*onDisable)();
  void (*onPidTune)(const String& key, float val);
  void (*onHoleStatus)(uint8_t row, uint8_t col, bool fertile);
  void (*onRevive)(const char* robotId);
  void (*onTestCommand)(const String& cmd);

  MQTTManager(const char* id)
    : boardId(id), enabled(false),
      onEnable(nullptr), onDisable(nullptr),
      onPidTune(nullptr), onHoleStatus(nullptr),
      onRevive(nullptr), onTestCommand(nullptr) {}

  void begin() {
    g_mqtt = this;
    messenger.onMessage(staticCallback);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD,
                    BROKER_HOST, BROKER_PORT,
                    GROUP_ID, boardId);
  }

  void loop() {
    messenger.loop();
  }

  bool isConnected() {
    return messenger.isConnected();
  }

  // ── Send helpers ──────────────────────────────────────────

  void sendPose(float x, float y, float heading) {
    char buf[64];
    snprintf(buf, sizeof(buf), "POSE:%.0f,%.0f,%.1f", x, y, heading);
    messenger.sendToGroup(buf);
  }

  void sendHoleStatus(uint8_t row, uint8_t col,
                       bool planted, bool fertile) {
    char buf[48];
    snprintf(buf, sizeof(buf), "HOLE:%d,%d,%s,%s",
             row, col,
             planted ? "1" : "0",
             fertile ? "1" : "0");
    messenger.sendToGroup(buf);
  }

  void sendState(const char* state) {
    char buf[64];
    snprintf(buf, sizeof(buf), "STATE:%s", state);
    messenger.sendToGroup(buf);
  }

  void sendLog(const char* msg) {
    char buf[128];
    snprintf(buf, sizeof(buf), "LOG:%s", msg);
    messenger.sendToGroup(buf);
  }

  void sendSensorSnapshot(const uint16_t irVals[], int centroid,
                           long udsL, long udsM, long udsR,
                           float heading) {
    // IR summary
    char buf[180];
    int pos = snprintf(buf, sizeof(buf),
                       "SENSOR:%d,%d,%d,%ld,%ld,%ld,%.1f",
                       centroid,
                       irVals[0], irVals[IR_COUNT - 1],
                       udsL, udsM, udsR, heading);
    messenger.sendToGroup(buf);
  }

  // ── Parse incoming messages ───────────────────────────────
  // Called from static callback.

  void handleMessage(const char* from, const char* msg) {
    // Kill switch
    if (strcmp(msg, "ENABLE") == 0) {
      enabled = true;
      Serial.println("MQTT: ENABLED");
      if (onEnable) onEnable();
      return;
    }
    if (strcmp(msg, "DISABLE") == 0) {
      enabled = false;
      Serial.println("MQTT: DISABLED");
      if (onDisable) onDisable();
      return;
    }

    // PID tuning: kp:1.5, ki:0.1, kd:0.05, md:35
    int sep = -1;
    for (int i = 0; msg[i]; i++) {
      if (msg[i] == ':') { sep = i; break; }
    }
    if (sep > 0 && sep < 10) {
      String key = String(msg).substring(0, sep);
      float val = String(msg).substring(sep + 1).toFloat();
      if (key == "kp" || key == "ki" || key == "kd" || key == "md") {
        if (onPidTune) onPidTune(key, val);
        return;
      }
    }

    // Hole status from server: HOLE_STATUS:row,col,fertile
    if (strncmp(msg, "HOLE_STATUS:", 12) == 0) {
      uint8_t r, c; int f;
      if (sscanf(msg + 12, "%hhu,%hhu,%d", &r, &c, &f) >= 3) {
        if (onHoleStatus) onHoleStatus(r, c, f != 0);
      }
      return;
    }

    // Revive request: REVIVE:robotId
    if (strncmp(msg, "REVIVE:", 7) == 0) {
      if (onRevive) onRevive(msg + 7);
      return;
    }

    // Test commands from dashboard: TEST:FOLLOW_LINE, TEST:FOLLOW_WALL, TEST:DEPOSIT
    if (strncmp(msg, "TEST:", 5) == 0) {
      if (onTestCommand) onTestCommand(String(msg + 5));
      return;
    }

    Serial.print("MQTT: unknown from ");
    Serial.print(from);
    Serial.print(": ");
    Serial.println(msg);
  }

private:
  static void staticCallback(const MessageMetadata& metadata,
                              const uint8_t* payload, size_t length) {
    if (!g_mqtt) return;
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    g_mqtt->handleMessage(metadata.fromBoardId, msg);
  }
};

#endif
