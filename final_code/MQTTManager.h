#ifndef MQTTMANAGER_H
#define MQTTMANAGER_H

#include <Arduino.h>
#include <MiniMessenger.h>
#include "secrets.h"
#include "Config.h"
#include "Map.h"

// ============================================================
//  MQTT MANAGER  —  MiniMessenger wrapper for robot comms
//  Handles standard server protocol (key=value pairs),
//  heartbeat watchdog, automatic registration, and
//  custom dashboard commands (PID tuning, test modes).
// ============================================================

class MQTTManager;

extern int g_seedIdx;

static MQTTManager* g_mqtt = nullptr;

static bool getKeyValue(const char* msg, const char* key,
                        char* out, size_t outLen) {
  size_t kLen = strlen(key);
  const char* p = msg;
  while (*p) {
    while (*p == ' ') p++;
    if (!*p) break;
    if (strncmp(p, key, kLen) == 0 && p[kLen] == '=') {
      p += kLen + 1;
      size_t vLen = 0;
      while (p[vLen] && p[vLen] != ' ') vLen++;
      size_t cp = (vLen < outLen - 1) ? vLen : outLen - 1;
      memcpy(out, p, cp);
      out[cp] = '\0';
      return true;
    }
    while (*p && *p != ' ') p++;
  }
  return false;
}

class MQTTManager {
public:
  MiniMessenger messenger;
  const char* boardId;

  // Priority-based enable: serverAllow (medium) && dashboardDesired (lowest).
  // Killed (highest) is owned by final_code.ino.
  bool serverAllow;
  bool dashboardDesired;

  // ── Callbacks ──────────────────────────────────────────────
  void (*onEnable)();
  void (*onDisable)();
  void (*onEmergency)();
  void (*onHoleStatus)(uint8_t row, uint8_t col, bool fertile, bool planted);
  void (*onRevive)(const char* robotId);
  void (*onTestCommand)(const String& cmd);
  void (*onPidTune)(const String& key, float val);
  void (*onHeadingReset)();
  void (*onAirlockReply)(bool accepted);
  void (*onSeedSelect)(int index);

  MQTTManager(const char* id)
    : boardId(id), serverAllow(false), dashboardDesired(false),
      onEnable(nullptr), onDisable(nullptr),
      onEmergency(nullptr), onHoleStatus(nullptr),
      onRevive(nullptr), onTestCommand(nullptr),
      onPidTune(nullptr), onHeadingReset(nullptr),
      onAirlockReply(nullptr), onSeedSelect(nullptr) {}

  // ── Priority model ─────────────────────────────────────────
  // serverAllow = medium (server heartbeat).
  // dashboardDesired = low (dashboard ENABLE/DISABLE commands).
  // killed (hardware, highest) is checked externally.
  // Effective = !killed && serverAllow && dashboardDesired.

  bool isEffectivelyEnabled() const {
    return serverAllow && dashboardDesired;
  }

  // Call when serverAllow or dashboardDesired may have changed.
  // Fires onEnable/onDisable if effective state toggles.
  void applyState() {
    bool eff = isEffectivelyEnabled();
    if (eff != lastEffective) {
      lastEffective = eff;
      if (eff) { Serial.println("MQTT: enabled");
                 if (onEnable) onEnable(); }
      else     { Serial.println("MQTT: disabled");
                 if (onDisable) onDisable(); }
    }
  }

  // ── Initialise ─────────────────────────────────────────────
  void begin() {
    g_mqtt = this;
    messenger.onMessage(staticCallback);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD,
                    BROKER_HOST, BROKER_PORT,
                    GROUP_ID, boardId);
  }

  // ── Poll (call every main loop iteration) ──────────────────
  void loop() {
    messenger.loop();

    // Heartbeat watchdog: auto-disallow if no heartbeat for 1s
    // Only fires after server has contacted us at least once.
    if (serverEverContacted && millis() - lastHeartbeatMs > 1000u) {
      serverAllow = false;
      Serial.println("MQTT: heartbeat timeout — server disallow");
      applyState();
    }

    // Registration every 10 seconds
    if (messenger.isConnected() && millis() - lastRegisterMs > 10000u) {
      lastRegisterMs = millis();
      char reg[80];
      snprintf(reg, sizeof(reg),
               "type=register team_id=%s board_id=%s",
               GROUP_ID, boardId);
      messenger.sendToBoard("server", reg);
    }
  }

  bool isConnected() {
    return messenger.isConnected();
  }

  // ── Standard Server Protocol: send ─────────────────────────

  void sendIsFertile(const char* tagId) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "type=isFertile tag_id=%s board_id=%s",
             tagId, boardId);
    messenger.sendToBoard("server", buf);
  }

  void sendSeedPlanted(const char* tagId) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "type=seedPlanted tag_id=%s board_id=%s",
             tagId, boardId);
    messenger.sendToBoard("server", buf);
  }

  void sendAirlockRequest(const char* airlockId, const char* tagId) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "type=openAirlock airlock=%s tag_id=%s board_id=%s",
             airlockId, tagId, boardId);
    messenger.sendToBoard("server", buf);
  }

  // ── Dashboard: directed to DASHBOARD_ID only (not all robots) ──

  void sendPose(float x, float y, float heading) {
    char buf[64];
    snprintf(buf, sizeof(buf), "POSE:%.0f,%.0f,%.1f", x, y, heading);
    messenger.sendToBoard(DASHBOARD_ID, buf);
  }

  void sendHoleStatus(uint8_t row, uint8_t col,
                       bool planted, bool fertile) {
    char buf[48];
    snprintf(buf, sizeof(buf), "HOLE:%d,%d,%s,%s",
             row, col,
             planted ? "1" : "0",
             fertile ? "1" : "0");
    messenger.sendToBoard(DASHBOARD_ID, buf);
  }

  void sendState(const char* state) {
    char buf[64];
    snprintf(buf, sizeof(buf), "STATE:%s", state);
    messenger.sendToBoard(DASHBOARD_ID, buf);
  }

  void sendLog(const char* msg) {
    char buf[144];
    snprintf(buf, sizeof(buf), "LOG:%s", msg);
    messenger.sendToBoard(DASHBOARD_ID, buf);
    Serial.print("LOG: ");
    Serial.println(msg);
  }

  void sendSensorSnapshot(const uint16_t irVals[], int centroid,
                           long udsL, long udsM, long udsR,
                           float heading, int lightVal) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "SENSOR:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%.1f,%d,%d",
             centroid,
             irVals[0], irVals[1], irVals[2], irVals[3], irVals[4],
             irVals[5], irVals[6], irVals[7], irVals[8],
             udsL, udsM, udsR, heading, lightVal, g_seedIdx);
    messenger.sendToBoard(DASHBOARD_ID, buf);
  }

  // ── Receive: parse incoming messages ───────────────────────

  void handleMessage(const char* from, const char* msg) {
    // ── isFertileReply — raw strncmp (matches working test script) ──
    if (strncmp(msg, "type=isFertileReply", 19) == 0) {
      char fertile[8] = "", planted[8] = "", xv[8] = "", yv[8] = "";
      const char* p = msg;
      while (*p) {
        while (*p == ' ') p++;
        if (strncmp(p, "fertile=", 8) == 0) { size_t j = 0; p += 8; while (*p && *p != ' ' && j < 7) fertile[j++] = *p++; fertile[j] = 0; continue; }
        if (strncmp(p, "planted=", 8) == 0) { size_t j = 0; p += 8; while (*p && *p != ' ' && j < 7) planted[j++] = *p++; planted[j] = 0; continue; }
        if (strncmp(p, "x=", 2) == 0) { size_t j = 0; p += 2; while (*p && *p != ' ' && j < 7) xv[j++] = *p++; xv[j] = 0; continue; }
        if (strncmp(p, "y=", 2) == 0) { size_t j = 0; p += 2; while (*p && *p != ' ' && j < 7) yv[j++] = *p++; yv[j] = 0; continue; }
        while (*p && *p != ' ') p++;
      }
      Serial.print(F("MQTT: isFertileReply fertile="));
      Serial.print(fertile);
      Serial.print(F(" planted="));
      Serial.print(planted);
      Serial.print(F(" x="));
      Serial.print(xv);
      Serial.print(F(" y="));
      Serial.println(yv);
      if (onHoleStatus && strlen(fertile) && strlen(planted) && strlen(xv) && strlen(yv)) {
        bool f = (strcmp(fertile, "true") == 0);
        bool p = (strcmp(planted, "true") == 0);
        onHoleStatus((uint8_t)atoi(yv), (uint8_t)atoi(xv), f, p);
      }
      return;
    }

    // Try standard key=value protocol
    char typeVal[32];
    if (getKeyValue(msg, "type", typeVal, sizeof(typeVal))) {

      if (strcmp(typeVal, "heartbeat") == 0) {
        lastHeartbeatMs = millis();
        serverEverContacted = true;
        char enableVal[4];
        if (getKeyValue(msg, "enable", enableVal, sizeof(enableVal))) {
          serverAllow = (strcmp(enableVal, "1") == 0);
          Serial.print("MQTT: heartbeat enable=");
          Serial.println(serverAllow ? "1" : "0");
          applyState();
        }
        return;
      }

      if (strcmp(typeVal, "emergency") == 0) {
        serverAllow = false;
        dashboardDesired = false;
        Serial.println("MQTT: EMERGENCY STOP");
        if (onEmergency) onEmergency();
        applyState();
        return;
      }

      if (strcmp(typeVal, "disable") == 0) {
        serverAllow = false;
        Serial.println("MQTT: disabled by server");
        applyState();
        return;
      }

      if (strcmp(typeVal, "openAirlockReply") == 0) {
        char acc[8];
        if (getKeyValue(msg, "accepted", acc, sizeof(acc))) {
          bool ok = (strcmp(acc, "true") == 0);
          Serial.print("MQTT: airlock ");
          Serial.println(ok ? "accepted" : "denied");
          if (onAirlockReply) onAirlockReply(ok);
        }
        return;
      }

      // Unknown standard message — fall through to legacy
    }

    // ── Legacy / Dashboard Commands ─────────────────────────
    if (strcmp(msg, "ENABLE") == 0) {
      // Dashboard enable (lowest priority) — only effective if server allows
      dashboardDesired = true;
      Serial.println("MQTT: dashboard wants ENABLE");
      applyState();
      return;
    }
    if (strcmp(msg, "DISABLE") == 0) {
      dashboardDesired = false;
      Serial.println("MQTT: dashboard wants DISABLE");
      applyState();
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

    // Hole status from server (legacy): HOLE_STATUS:row,col,fertile
    if (strncmp(msg, "HOLE_STATUS:", 12) == 0) {
      uint8_t r, c; int f;
      if (sscanf(msg + 12, "%hhu,%hhu,%d", &r, &c, &f) >= 3) {
        // Map legacy format to new callback
        if (onHoleStatus) onHoleStatus(r, c, f != 0, false);
      }
      return;
    }

    // Revive request: REVIVE:robotId
    if (strncmp(msg, "REVIVE:", 7) == 0) {
      if (onRevive) onRevive(msg + 7);
      return;
    }

    // Reset heading heading
    if (strcmp(msg, "HEADING:0") == 0 || strcmp(msg, "RESET_HEADING") == 0) {
      if (onHeadingReset) onHeadingReset();
      return;
    }

    // Seed selection: SEED:1 through SEED:5
    if (strncmp(msg, "SEED:", 5) == 0) {
      int idx = atoi(msg + 5);
      if (idx >= 0 && idx < SEED_COUNT && onSeedSelect) onSeedSelect(idx);
      return;
    }

    // Test commands: TEST:FOLLOW_LINE, TEST:FOLLOW_WALL, TEST:DEPOSIT
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
  unsigned long lastHeartbeatMs = 0;
  unsigned long lastRegisterMs = 0;
  bool lastEffective = false;
  bool serverEverContacted = false;

  static void staticCallback(const MessageMetadata& metadata,
                              const uint8_t* payload, size_t length) {
    if (!g_mqtt) return;
    char msg[256];
    size_t copyLen = (length < 255) ? length : 255;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';
    g_mqtt->handleMessage(metadata.fromBoardId, msg);
  }
};

#endif
