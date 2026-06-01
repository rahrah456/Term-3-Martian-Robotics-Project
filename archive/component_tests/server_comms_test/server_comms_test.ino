// ============================================================
//  SERVER COMMS TEST
//  Standalone sketch to test all robot ↔ server interactions
//  by manually moving the robot over RFID tags.
//
//  Instructions:
//    1. Open Serial Monitor at 115200 baud.
//    2. Wait for "Connected!" and "Register sent."
//    3. Place robot over an RFID tag — tag ID prints automatically.
//    4. Type a command and press Enter (or just type the letter):
//         f  — isFertile query (planting sequence)
//         p  — seedPlanted confirmation
//         a  — openAirlock A (base exit)
//         b  — openAirlock B (tunnel exit)
//         r  — re-register with server
//         h  — print this help
//    5. Server replies are printed in real-time with parsed fields.
// ============================================================

#include <Arduino.h>
#include <MiniMessenger.h>
#include <Wire.h>
#include <MFRC522_I2C.h>

// ── Credentials ──────────────────────────────────────────────
const char* WIFI_SSID  = "PhaseSpaceNetwork_2.4G";
const char* WIFI_PASS  = "8igMacNet";
const char* BROKER     = "192.168.0.74";
const uint16_t PORT    = 1883;
const char* GROUP_ID   = "3";
const char* BOARD_ID   = "Haunter";
const char* DASHBOARD  = "dash3";

// ── RFID ─────────────────────────────────────────────────────
MFRC522_I2C rfid(0x28, -1, &Wire1);
char lastTagId[20] = "";
unsigned long lastTagMs = 0;

// ── Throttle ─────────────────────────────────────────────────
unsigned long lastRcvPrint = 0;

// ── MQTT ─────────────────────────────────────────────────────
MiniMessenger messenger;

void onMessage(const MessageMetadata& meta,
               const uint8_t* payload, size_t len);

// ── Help text ───────────────────────────────────────────────
void printHelp() {
  Serial.println(F(""));
  Serial.println(F("── Commands ──────────────────────"));
  Serial.println(F("  f  — isFertile     (planting)"));
  Serial.println(F("  p  — seedPlanted   (confirm plant)"));
  Serial.println(F("  a  — openAirlock A (base exit)"));
  Serial.println(F("  b  — openAirlock B (tunnel exit)"));
  Serial.println(F("  r  — register"));
  Serial.println(F("  h  — this help"));
  Serial.println(F("──────────────────────────────────"));
}

// ── Send functions ──────────────────────────────────────────
void sendIsFertile() {
  if (strlen(lastTagId) == 0) {
    Serial.println(F("No tag read yet. Move robot over an RFID tag first."));
    return;
  }
  char buf[100];
  snprintf(buf, sizeof(buf),
           "type=isFertile tag_id=%s board_id=%s",
           lastTagId, BOARD_ID);
  messenger.sendToBoard("server", buf);
  Serial.print(F("[SENT] isFertile  tag_id="));
  Serial.println(lastTagId);
}

void sendSeedPlanted() {
  if (strlen(lastTagId) == 0) {
    Serial.println(F("No tag read yet."));
    return;
  }
  char buf[100];
  snprintf(buf, sizeof(buf),
           "type=seedPlanted tag_id=%s board_id=%s",
           lastTagId, BOARD_ID);
  messenger.sendToBoard("server", buf);
  Serial.print(F("[SENT] seedPlanted  tag_id="));
  Serial.println(lastTagId);
}

void sendOpenAirlock(char airlockId) {
  if (strlen(lastTagId) == 0) {
    Serial.println(F("No tag read yet."));
    return;
  }
  char buf[100];
  snprintf(buf, sizeof(buf),
           "type=openAirlock airlock=%c tag_id=%s board_id=%s",
           airlockId, lastTagId, BOARD_ID);
  messenger.sendToBoard("server", buf);
  Serial.print(F("[SENT] openAirlock "));
  Serial.print(airlockId);
  Serial.print(F("  tag_id="));
  Serial.println(lastTagId);
}

void sendRegister() {
  char buf[80];
  snprintf(buf, sizeof(buf),
           "type=register team_id=%s board_id=%s",
           GROUP_ID, BOARD_ID);
  messenger.sendToBoard("server", buf);
  Serial.println(F("[SENT] register"));
}

// ── Read RFID (debounced) ───────────────────────────────────
bool readRFIDTag(char* buf, size_t len) {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;

  size_t idx = 0;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10 && idx < len - 1) buf[idx++] = '0';
    int n = snprintf(&buf[idx], len - idx, "%X", rfid.uid.uidByte[i]);
    if (n <= 0) break;
    idx += n;
    if (idx >= len - 1) break;
  }
  buf[idx] = '\0';

  // Debounce: skip same tag within 2 seconds
  if (strcmp(buf, lastTagId) == 0 && millis() - lastTagMs < 2000)
    return false;

  snprintf(lastTagId, sizeof(lastTagId), "%s", buf);
  lastTagMs = millis();
  return true;
}

// ── Incoming message handler ────────────────────────────────
void onMessage(const MessageMetadata& meta,
               const uint8_t* payload, size_t len) {
  bool showRcv = (millis() - lastRcvPrint > 1000);
  if (showRcv) lastRcvPrint = millis();

  // ── Team status broadcast (6 bytes, ~1Hz) ──
  if (len == 6) {
    if (showRcv) {
      Serial.print(F("[TEAM STATUS]  queueExit="));
      Serial.print(payload[0]);
      Serial.print(F("  airlockBBusy="));
      Serial.print(payload[1]);
      Serial.print(F("  queueEnter="));
      Serial.print(payload[2]);
      Serial.print(F("  airlockABusy="));
      Serial.print(payload[3]);
      Serial.print(F("  emergency="));
      Serial.print(payload[4]);
      Serial.print(F("  reEntryRequested="));
      Serial.println(payload[5]);
    }
    return;
  }

  // ── Occupancy map (21 bytes) ──
  if (len == 21) {
    Serial.print(F("[MAP] 21 bytes received"));
    Serial.print(F("  first="));
    if (payload[0] < 0x10) Serial.print(F("0"));
    Serial.print(payload[0], HEX);
    Serial.print(F("  last="));
    if (payload[20] < 0x10) Serial.print(F("0"));
    Serial.println(payload[20], HEX);
    return;
  }

  // ── Text messages ──
  char msg[256];
  size_t copyLen = (len < 255) ? len : 255;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  // ── Parse known replies ──

  // isFertileReply  e.g.
  //   type=isFertileReply fertile=true planted=false x=6 y=7
  if (strncmp(msg, "type=isFertileReply", 19) == 0) {
    if (showRcv) { Serial.print(F("[RCV] from ")); Serial.print(meta.fromBoardId); Serial.print(F(": ")); Serial.println(msg); }
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
    Serial.print(F("  → fertile="));
    Serial.print(fertile);
    Serial.print(F("  planted="));
    Serial.print(planted);
    Serial.print(F("  x="));
    Serial.print(xv);
    Serial.print(F("  y="));
    Serial.println(yv);
    return;
  }

  // openAirlockReply  e.g.
  //   type=openAirlockReply airlock=A accepted=true queue_enter=0 queue_exit=0
  if (strncmp(msg, "type=openAirlockReply", 21) == 0) {
    if (showRcv) { Serial.print(F("[RCV] from ")); Serial.print(meta.fromBoardId); Serial.print(F(": ")); Serial.println(msg); }
    if (strstr(msg, "accepted=true")) {
      Serial.println(F("  → AIRLOCK ACCEPTED"));
    } else if (strstr(msg, "accepted=false")) {
      Serial.println(F("  → AIRLOCK DENIED"));
      const char* reason = strstr(msg, "reason=");
      if (reason) {
        Serial.print(F("     reason: "));
        reason += 7;
        while (*reason && *reason != ' ') Serial.write(*reason++);
        Serial.println();
      }
    }
    return;
  }

  // heartbeat — only print raw line (throttled), no parsed fields
  if (strncmp(msg, "type=heartbeat", 14) == 0) {
    if (showRcv) { Serial.print(F("[RCV] from ")); Serial.print(meta.fromBoardId); Serial.print(F(": ")); Serial.println(msg); }
    return;
  }

  // Everything else — print raw line if not throttled
  if (showRcv) {
    Serial.print(F("[RCV] from "));
    Serial.print(meta.fromBoardId);
    Serial.print(F(": "));
    Serial.println(msg);
  }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println(F(""));
  Serial.println(F("╔═══════════════════════════════════╗"));
  Serial.println(F("║      SERVER COMMS TEST            ║"));
  Serial.println(F("╚═══════════════════════════════════╝"));
  Serial.println(F(""));

  // RFID
  Wire1.begin();
  rfid.PCD_Init();
  Serial.println(F("RFID ready"));

  // MQTT
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASS, BROKER, PORT, GROUP_ID, BOARD_ID);

  Serial.print(F("Connecting to WiFi/MQTT"));
  unsigned long t0 = millis();
  while (!messenger.isConnected() && millis() - t0 < 20000) {
    messenger.loop();
    delay(300);
    Serial.print(F("."));
  }
  Serial.println(F(""));
  if (messenger.isConnected()) {
    Serial.println(F("Connected!"));
    sendRegister();
  } else {
    Serial.println(F("Connection FAILED — check WiFi credentials and broker"));
  }

  printHelp();
  Serial.println(F("Move robot over an RFID tag to start."));
  Serial.println(F(""));
}

// ── Main loop ───────────────────────────────────────────────
void loop() {
  messenger.loop();

  // Read RFID
  char tagBuf[20];
  if (readRFIDTag(tagBuf, sizeof(tagBuf))) {
    Serial.print(F("\n>>> New tag: "));
    Serial.println(lastTagId);
    Serial.println(F("    f=fertile  p=planted  a=airlock A  b=airlock B  r=register  h=help"));
  }

  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) { Serial.read(); delay(5); }

    switch (c) {
      case 'f': case 'F': sendIsFertile();      break;
      case 'p': case 'P': sendSeedPlanted();    break;
      case 'a': case 'A': sendOpenAirlock('A'); break;
      case 'b': case 'B': sendOpenAirlock('B'); break;
      case 'r': case 'R': sendRegister();       break;
      case 'h': case 'H': printHelp();          break;
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(c);
        printHelp();
        break;
    }
  }
}
