// ============================================================
//  FINAL CODE  —  Martian Robotics Project  (Term 3)
//  Single M7 sketch: sensors, localisation, state machine, MQTT
//  All motion is non-blocking; the main loop ticks MotionSM.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>

#include "Config.h"
#include "Map.h"
#include "Sensors.h"
#include "Control.h"
#include "Localisation.h"
#include "MQTTManager.h"

// ── Hardware objects ────────────────────────────────────────
MotoronI2C mc(0x10);
Servo servo;
MFRC522_I2C rfid(0x28, -1, &Wire1);

// ── Modules ─────────────────────────────────────────────────
ArenaMap arena;
Localisation loc;
IMUData imuData;
MQTTManager mqtt("Haunter2");
MotionSM motion;                   // non-blocking motion controller
ReviveMove revive;                 // decelerating open-loop push

// ── Encoder polling (50 Hz, replaces ISR-based to avoid WiFi interference) ─
// Reads both channels of each encoder and advances a full quadrature
// state machine.  4× resolution vs single-edge ISR, no interrupts needed.
volatile long encL = 0, encR = 0;

static const int8_t QUAD_TABLE[16] = {
  0,  1, -1,  0,
 -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

void pollEncoders() {
  static int prevL = 0, prevR = 0;
  int aL = digitalRead(PIN_ENC_LA);
  int bL = digitalRead(PIN_ENC_LB);
  int aR = digitalRead(PIN_ENC_RA);
  int bR = digitalRead(PIN_ENC_RB);
  int stateL = (aL << 1) | bL;
  int stateR = (aR << 1) | bR;
  encL += QUAD_TABLE[(prevL << 2) | stateL];
  encR += QUAD_TABLE[(prevR << 2) | stateR];
  prevL = stateL;
  prevR = stateR;
}

// ── UDS after median (UDSManager) + EMA smoothing ─────────
// EMA alpha = 0.2 → ~5-tick response (100ms at 50Hz)
static EMA udsLFilter(0.2f), udsMFilter(0.2f), udsRFilter(0.2f);
float filteredUdsL, filteredUdsM, filteredUdsR;
volatile bool killed = false;
bool motorsRunning = false;
volatile bool motorTestStop = false;

// ── State Machine ───────────────────────────────────────────
enum State {
  ST_INIT, ST_IDLE, ST_LOCATE, ST_PLAN,
  ST_NAVIGATE, ST_AVOID, ST_DEPOSIT,
  ST_RETURN_BASE, ST_REVIVE, ST_LET_IN,
  ST_EXIT_BASE, ST_TEST
};

const char* stateNames[] = {
  "INIT", "IDLE", "LOCATE", "PLAN",
  "NAVIGATE", "AVOID", "DEPOSIT",
  "RETURN_BASE", "REVIVE", "LET_IN",  
  "EXIT_BASE", "TEST"
};

State state = ST_INIT;

// ── Turn multiplier overrides (tuneable from dashboard) ────
#define MAX_TURN_MULTS 8
struct TurnMultSet {
  float vals[MAX_TURN_MULTS];
  int count = 0, idx = 0;
  void reset() { idx = 0; }
  float next() { return (idx < count) ? vals[idx++] : 1.0f; }
  void parse(const char* s) {
    count = 0;
    char buf[128]; strncpy(buf, s, sizeof(buf)-1); buf[sizeof(buf)-1] = 0;
    char* tok = strtok(buf, ",");
    while (tok && count < MAX_TURN_MULTS) { vals[count++] = (float)atof(tok); tok = strtok(NULL, ","); }
  }
};
TurnMultSet g_gridMults;

bool handleEStop() {
  static bool lastBtn = HIGH;
  static unsigned long debounce = 0;
  bool btn = digitalRead(PIN_KILL_BTN);
  if (btn == LOW && lastBtn == HIGH && millis() - debounce > 50) {
    debounce = millis();
    killed = !killed;
    if (killed) { setMotors(mc, 0, 0); digitalWrite(PIN_ACT_LED, HIGH); mqtt.sendState("KILLED"); }
    else        { digitalWrite(PIN_ACT_LED, LOW); mqtt.sendState(stateNames[state]); }
  }
  lastBtn = btn;
  return killed;
}

void waitForUnkill() {
  while (killed) { handleEStop(); mqtt.loop(); delay(20); }
}

int waitForMotion() {
  unsigned long _mqttDead = millis() + 5;
  unsigned long _encLast = micros();
  int _ticks = 0;
  while (true) {
    unsigned long _now = micros();
    if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); _ticks++; }
    if ((long)(millis() - _mqttDead) >= 0) { mqtt.loop(); _mqttDead = millis() + 5; }
    int mr = motion.tick(mc);
    if (mr != MotionSM::RUNNING) { Serial.print("waitForMotion: tick returned "); Serial.println(mr); return mr; }
    handleEStop(); if (killed) { Serial.println("waitForMotion: killed"); motion.stop(); setMotors(mc, 0, 0); return MotionSM::DONE; }
    if (!mqtt.isEffectivelyEnabled()) { Serial.println("waitForMotion: disabled"); motion.stop(); setMotors(mc, 0, 0); return MotionSM::DONE; }
  }
}

// ── RFID read ────────────────────────────────────────────────
bool readRFID(char* buf, size_t len) {
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
  return true;
}

// ── PID tuning params (tuneable via MQTT) ───────────────────
float pidKp = 30, pidKi = 1.0, pidKd = 0.0;
int   pidMaxDiff = 120;

// ── Hole memory ─────────────────────────────────────────────
bool holePlanted[GRID_HOLES][GRID_HOLES] = {false};
bool holeFertile[GRID_HOLES][GRID_HOLES];

// ── Navigation target ──────────────────────────────────────
PointI navTarget = {0, 0};
uint8_t navTargetRow = 0, navTargetCol = 0;

// ── Timing ──────────────────────────────────────────────────
unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
unsigned long startTime = 0;

// ── UDS manager (non-blocking round-robin) ──────────────────
UDSManager uds;

// ── Sensor state ────────────────────────────────────────────
uint16_t irVals[IR_COUNT];
int irCentroidVal = -1;
long udsL, udsM, udsR;   // latest median-filtered readings
int lightVal;
char rfidBuf[32];
bool rfidSeen = false;
int8_t lastHoleReplyRow = -1, lastHoleReplyCol = -1;
int  g_seedIdx = 1;   // current servo index, 1 by default (all 5 seeds loaded)
bool airlockAccepted = false;

// ============================================================
//  MQTT CALLBACKS
// ============================================================

void onMqttEnable() {
  killed = false;
  digitalWrite(PIN_ACT_LED, LOW);
  mqtt.sendLog("enabled");
  mqtt.sendState(stateNames[state]);
}

void onMqttDisable() {
  setMotors(mc, 0, 0);
  digitalWrite(PIN_ACT_LED, HIGH);
  mqtt.sendState("DISABLED");
}

void onMqttPidTune(const String& key, float val) {
  if (key == "kp")       { pidKp = val; motion.kp = val; mqtt.sendLog("kp set"); }
  else if (key == "ki")  { pidKi = val; motion.ki = val; mqtt.sendLog("ki set"); }
  else if (key == "kd")  { pidKd = val; motion.kd = val; mqtt.sendLog("kd set"); }
  else if (key == "md")  { pidMaxDiff = constrain((int)val, 0, 130); motion.maxDiff = pidMaxDiff; mqtt.sendLog("md set"); }
}

void onMqttHoleStatus(uint8_t row, uint8_t col, bool fertile, bool planted) {
  if (row < GRID_HOLES && col < GRID_HOLES) {
    holeFertile[row][col] = fertile;
    if (planted) holePlanted[row][col] = true;
    lastHoleReplyRow = row;
    lastHoleReplyCol = col;
  }
}

void onMqttEmergency() {
  setMotors(mc, 0, 0);
  motion.stop();
  digitalWrite(PIN_ACT_LED, HIGH);
  mqtt.sendLog("EMERGENCY STOP");
}

void onMqttRevive(const char* robotId) {
  mqtt.sendLog("revive requested");
}

void onMqttHeadingReset() {
  loc.resetHeading(0.0f);
  imuData.headingDeg = 0.0f;
  mqtt.sendLog("heading reset to 0");
}

void onMqttAirlockReply(bool accepted) {
  airlockAccepted = accepted;
  mqtt.sendLog(accepted ? "airlock accepted" : "airlock denied");
}

void onMqttSeedSelect(int idx) {
  if (idx < 0 || idx >= SEED_COUNT) return;
  dispenseSeed(servo, idx);
  mqtt.sendLog("seed pos set");
}

// ── Test mode: local blocking loop with full sensor reads ───
// Blocks inside a while(1) so sensors, MQTT, and IMU all stay
// hot.  The motion controller advances one step per pass.

static void runTestLoop(unsigned long durationMs) {
  lastPublish = 0; // force instant publish on first pass
  unsigned long endMs = (durationMs == 0) ? 0xFFFFFFFF : millis() + durationMs;
  while (millis() < endMs) {
    mqtt.loop();
    if (handleEStop()) { setMotors(mc, 0, 0); motion.stop(); waitForUnkill(); break; }
    if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); motion.stop(); break; }
    if (motorTestStop) { motorTestStop = false; setMotors(mc, 0, 0); motion.stop(); break; }

    // Full sensor read
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    pollEncoders();
    uds.tick();
    udsL = uds.distances[UDSManager::LEFT];
    udsM = uds.distances[UDSManager::MID];
    udsR = uds.distances[UDSManager::RIGHT];
    if (imuData.ok) readIMU(imuData);
    loc.update(encL, encR, imuData.gyroZ, imuData.accX, imuData.accY, imuData.pitch, imuData.roll);
    if (imuData.ok) imuData.headingDeg = loc.pose.headingDeg;

    // Publish every 500ms
    if (millis() - lastPublish >= 500) {
      lastPublish = millis();
      mqtt.sendPose(loc.pose.x, loc.pose.y, loc.pose.headingDeg);
      mqtt.sendState("TEST");
      mqtt.sendSensorSnapshot(irVals, irCentroidVal, udsL, udsM, udsR, imuData.headingDeg, lightVal);
    }

    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  setMotors(mc, 0, 0);
  motion.stop();
}

void onMqttTestCommand(const String& cmd) {
  mqtt.sendLog("cmd received");
  Serial.print("Test: "); Serial.println(cmd);

  if (cmd.startsWith("FOLLOW_LINE")) {
    int base = 500;
    float p = pidKp, i = pidKi, d = pidKd;
    int md = pidMaxDiff;
    sscanf(cmd.c_str(), "FOLLOW_LINE:%d,%f,%f,%f,%d", &base, &p, &i, &d, &md);
    motion.startLineFollow(base, p, i, d, md, 20000);
    state = ST_TEST;
    runTestLoop(0);
    state = ST_IDLE;
    mqtt.sendLog("line follow done");
  }
  else if (cmd.startsWith("FOLLOW_WALL")) {
    int base = 500, side = 1;
    float targetCm = 8.0;
    float p = 1.5, i = 0.0, d = 0.0;
    int md = pidMaxDiff;
    sscanf(cmd.c_str(), "FOLLOW_WALL:%d,%d,%f,%f,%f,%f,%d", &base, &side, &targetCm, &p, &i, &d, &md);
    motion.startWallFollow(base, side, targetCm, p, i, d, md, 10000);
    state = ST_TEST;
    runTestLoop(10000);
    state = ST_IDLE;
    mqtt.sendLog("wall follow done");
  }
  else if (cmd.startsWith("OBSTACLE_AVOIDANCE")) {
    mqtt.sendLog("Testing obstacle avoidance detour...");
    state = ST_AVOID;
    runAvoid();
    state = ST_IDLE;
  }
  else if (cmd.startsWith("MOVE_TURN")) {
    int mm = 250, deg = 0;
    sscanf(cmd.c_str(), "MOVE_TURN:%d,%d", &mm, &deg);
    char _mt[64]; snprintf(_mt, sizeof(_mt), "move_turn: %dmm, %ddeg", mm, deg); mqtt.sendLog(_mt);
    motion.startStraight(MOVE_SPEED, ticksForDistance(mm));
    waitForMotion(); if (killed) return;
    if (deg != 0) {
      motion.startTurn(deg > 0 ? 1 : -1, TURN_SPEED, ticksForTurn(abs(deg)));
      waitForMotion(); if (killed) return;
    }
    mqtt.sendLog("move_turn: done");
  }
  else if (cmd == "DEPOSIT") {
    // Read fresh sensors — callback runs from mqtt.loop() context
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (imuData.ok) readIMU(imuData);
    pollEncoders();
    state = ST_TEST;
    mqtt.sendLog("deposit: advancing 6cm");
    motion.startStraight(MOVE_SPEED, ticksForDistance(60));
    waitForMotion();
    runDeposit();
    state = ST_IDLE;
    mqtt.sendLog("deposit done");
  }
  else if (cmd == "EXIT_BASE") {
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (imuData.ok) readIMU(imuData);
    pollEncoders();
    state = ST_TEST;
    runBaseExit();
    state = ST_IDLE;
    mqtt.sendLog("base exit done");
  }
  else if (cmd == "REVIVE") {
    float distMm = 2.0f * HOLE_SPACING_MM - CHASSIS_LENGTH;
    pollEncoders();
    revive.start(ticksForDistance(distMm));
    state = ST_TEST;
    while (revive.tick(mc) == MotionSM::RUNNING) {
      unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } }
      mqtt.loop();
      if (handleEStop()) { revive.stop(); setMotors(mc, 0, 0); break; }
      if (!mqtt.isEffectivelyEnabled()) { revive.stop(); setMotors(mc, 0, 0); break; }
    }
    state = ST_IDLE;
    mqtt.sendLog("revive done");
  }
  else if (cmd == "GRID_NAV") {
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (imuData.ok) readIMU(imuData);
    pollEncoders();
    state = ST_TEST;
    runGridNav();
    state = ST_IDLE;
    mqtt.sendLog("grid nav done");
  }
  else if (cmd == "GRID_NAV_NOLINES") {
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (imuData.ok) readIMU(imuData);
    pollEncoders();
    state = ST_TEST;
    runGridNavNoLines();
    state = ST_IDLE;
    mqtt.sendLog("grid nav nolines done");
  }
  else if (cmd.startsWith("OVERRIDE_GRID_TURNS:")) {
    g_gridMults.parse(cmd.c_str() + 20);
    char lb[64]; snprintf(lb, sizeof(lb), "grid mults: %d values", g_gridMults.count);
    mqtt.sendLog(lb);
  }
  else if (cmd.startsWith("MOTOR:L,") || cmd.startsWith("MOTOR:R,") || cmd.startsWith("MOTOR:BOTH,")) {
    char motor; int speed, l = 0, r = 0; unsigned long dur = 2000;
    if (cmd.startsWith("MOTOR:BOTH,")) {
      int n = sscanf(cmd.c_str(), "MOTOR:BOTH,%d,%d,%lu", &l, &r, &dur);
      if (n < 2) {
        sscanf(cmd.c_str(), "MOTOR:BOTH,%d,%lu", &speed, &dur);
        l = speed; r = speed;
      }
      motor = 'B';
    } else {
      sscanf(cmd.c_str(), "MOTOR:%c,%d,%lu", &motor, &speed, &dur);
      if (motor == 'L') l = speed;
      else if (motor == 'R') r = speed;
    }
    char lb[64]; snprintf(lb, sizeof(lb), "motor %c: L=%d R=%d, %lums", motor, l, r, dur);
    mqtt.sendLog(lb);
    motorTestStop = false;
    setMotors(mc, l, r);
    state = ST_TEST;
    runTestLoop(dur);
    state = ST_IDLE;
    mqtt.sendLog("motor test done");
  }
  else if (cmd == "MOTOR:STOP") {
    motorTestStop = true;
    setMotors(mc, 0, 0);
    mqtt.sendLog("motor stop");
  }
}

// ============================================================
//  STATE BEHAVIOURS
// ============================================================

void runInit() {
  Serial.println("State: INIT — waiting for ENABLE");
  mqtt.sendState("INIT");
  state = ST_IDLE;
}

void runIdle() {
  static unsigned long lastPlan = 0;
  setMotors(mc, 0, 0);

  if (loc.valid && millis() - lastPlan > 1000) {
    lastPlan = millis();
    state = ST_PLAN;
  }
}

void runPlan() {
  mqtt.sendState("PLAN");

  int bestIdx = -1;
  float bestDist = 1e9;
  for (uint8_t r = 0; r < GRID_HOLES; r++) {
    for (uint8_t c = 0; c < GRID_HOLES; c++) {
      if (holePlanted[r][c]) continue;
      PointI pt = arena.holeCentre(r, c);
      float d = loc.distanceTo(pt.x, pt.y);
      if (d < bestDist) { bestDist = d; bestIdx = r * GRID_HOLES + c; }
    }
  }

  if (bestIdx < 0) { state = ST_RETURN_BASE; mqtt.sendLog("all holes planted"); return; }

  navTargetRow = bestIdx / GRID_HOLES;
  navTargetCol = bestIdx % GRID_HOLES;
  navTarget = arena.holeCentre(navTargetRow, navTargetCol);
  // state = ST_NAVIGATE; // disabled navigating until Map.h is filled in and we can do that properly.
}

// ── Navigation (non-blocking, runs every loop) ──────────────
void runNavigate() {
  if (!loc.valid) { state = ST_IDLE; return; }

  // Pick target if none set
  if (navTarget.x == 0 && navTarget.y == 0) {
    int bestIdx = -1;
    float bestDist = 1e9;
    for (uint8_t r = 0; r < GRID_HOLES; r++)
      for (uint8_t c = 0; c < GRID_HOLES; c++) {
        if (holePlanted[r][c]) continue;
        PointI pt = arena.holeCentre(r, c);
        float d = loc.distanceTo(pt.x, pt.y);
        if (d < bestDist) { bestDist = d; bestIdx = r * GRID_HOLES + c; }
      }
    if (bestIdx < 0) { state = ST_RETURN_BASE; return; }
    navTargetRow = bestIdx / GRID_HOLES;
    navTargetCol = bestIdx % GRID_HOLES;
    navTarget = arena.holeCentre(navTargetRow, navTargetCol);
  }

  if (loc.distanceTo(navTarget.x, navTarget.y) < HOLE_SPACING_MM * 0.4f) {
    mqtt.sendLog("arrived at hole");
    state = ST_DEPOSIT;
    return;
  }

  if (obstacleAhead(filteredUdsM)) { setMotors(mc, 0, 0); state = ST_AVOID; return; }

  int baseSpeed = constrain(MOTOR_MIN + 80, MOTOR_MIN, MOTOR_MAX);
  bool lineVisible = (irCentroidVal >= 0);

  if (lineVisible) {
    float error = (float)irCentroidVal - 4000.0f;
    float correction = pidKp * error;
    correction = constrain(correction, -(float)pidMaxDiff, (float)pidMaxDiff);
    int left  = constrain(baseSpeed + (int)correction, MOTOR_MIN, MOTOR_MAX);
    int right = constrain(baseSpeed - (int)correction, MOTOR_MIN, MOTOR_MAX);
    setMotors(mc, left, right);
  } else {
    float bearing = loc.bearingTo(navTarget.x, navTarget.y);
    int steer = constrain((int)(bearing * 2.0f), -pidMaxDiff, pidMaxDiff);
    int left  = constrain(baseSpeed + steer, MOTOR_MIN, MOTOR_MAX);
    int right = constrain(baseSpeed - steer, MOTOR_MIN, MOTOR_MAX);
    setMotors(mc, left, right);
  }
}

// ── Drive forward with encoder-based distance stop ─────────
// Checks RFID every iteration after first 70mm — stops early if tag found.
static void driveDist(long ticks) {
  motion.startStraight(MOVE_SPEED, ticks);
  unsigned long _mDead = millis() + 5, _eLast = micros();
  unsigned long _snapLast = 0;
  long sL = encL, sR = encR;
  long rfidStart = ticksForDistance(70);
  bool canRFID = false;
  while (true) {
    unsigned long _n = micros();
    if (_n - _eLast >= 500) { _eLast = _n; pollEncoders(); }
    if ((long)(millis() - _mDead) >= 0) { mqtt.loop(); _mDead = millis() + 5; }
    if (millis() - _snapLast >= 200) { _snapLast = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    long d = (abs(encL - sL) + abs(encR - sR)) / 2;
    if (!canRFID && d >= rfidStart) canRFID = true;
    if (canRFID && readRFID(rfidBuf, sizeof(rfidBuf))) { setMotors(mc, 0, 0); motion.stop(); mqtt.sendLog("tag"); break; }
    int mr = motion.tick(mc);
    if (mr != MotionSM::RUNNING) break;
    handleEStop(); if (killed) { motion.stop(); setMotors(mc, 0, 0); return; }
    if (d >= ticks) { setMotors(mc, 0, 0); motion.stop(); break; }
  }
}

// ── Obstacle Avoidance (blocking, like runBaseExit) ─────────
// Three-phase sensor-guided detour:
//   Phase 0: reverse until front UDS=15cm, then spin 35° CCW
//   Phase 1: drive straight until side UDS <= 10cm, then hug 10-20cm diff
//   Phase 2: straight 3cm then arc CW to front of obstacle
//   Phase 3: rotate 20° CW, drive 40cm, then sweep search for IR line (centroid >= 0)

void runAvoid() {
  mqtt.sendState("AVOID");

  unsigned long phaseStart, encLast, checkLast;
  unsigned long sweepToggle;
  unsigned long lastPub = 0;
  int sweepDir = 1;

  // ── Phase 0: reverse until front UDS = 15cm, then spin 35° CCW ──
  mqtt.sendLog("avoid: phase 0 reverse");
  phaseStart = millis();
  encLast = micros();
  checkLast = millis();
  setMotors(mc, -300, -300);
  while (millis() - phaseStart < 5000) {
    unsigned long now = micros();
    if (now - encLast >= 500) { encLast = now; pollEncoders(); }
    if (millis() - checkLast >= 5) { checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) { setMotors(mc, 0, 0); return; } }
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (millis() - lastPub >= 200) { lastPub = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    if (filteredUdsM >= 15.0f) break;
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  if (killed) return;
  setMotors(mc, 0, 0);
  mqtt.sendLog("avoid: phase 0 spin 35");
  long p0SpinStartL = encL, p0SpinStartR = encR;
  long p0Target = ticksForTurn(38);
  phaseStart = millis();
  encLast = micros();
  checkLast = millis();
  setMotors(mc, -TURN_SPEED, TURN_SPEED);
  while (millis() - phaseStart < 5000) {
    unsigned long now = micros();
    if (now - encLast >= 500) { encLast = now; pollEncoders(); }
    if (millis() - checkLast >= 5) { checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) { setMotors(mc, 0, 0); return; } }
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (((long)abs(encL - p0SpinStartL) + (long)abs(encR - p0SpinStartR)) / 2 >= p0Target) { setMotors(mc, 0, 0); break; }
    if (millis() - lastPub >= 200) { lastPub = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  if (killed) return;
  mqtt.sendLog("avoid: phase 0 done");

  // ── Phase 1: drive straight until side within 10cm, then hug 10-20 ──
  mqtt.sendLog("avoid: phase 1 approach");
  encLast = micros();
  checkLast = millis();
  unsigned long p1Deadline = millis() + 10000;
  bool p1Hugging = false;
  while (millis() < p1Deadline) {
    unsigned long now = micros();
    if (now - encLast >= 500) { encLast = now; pollEncoders(); }
    if (millis() - checkLast >= 5) { checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) { setMotors(mc, 0, 0); return; } }
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (millis() - lastPub >= 200) { lastPub = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    if (filteredUdsR > 50.0f) break;
    if (!p1Hugging && filteredUdsR <= 10.0f) p1Hugging = true;
    if (p1Hugging) {
      float error = 15.0f - filteredUdsR;
      int correction = constrain((int)(pidKp * 0.4f * error), -pidMaxDiff, pidMaxDiff);
      setMotors(mc, 350 + correction, 350 - correction);
    } else {
      setMotors(mc, 350, 350);
    }
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  if (killed) return;
  setMotors(mc, 0, 0);
  mqtt.sendLog("avoid: phase 1 done");
  mqtt.sendLog("avoid: phase 1 spin cw 28");
  motion.startTurn(1, TURN_SPEED, ticksForTurn(28));
  waitForMotion(); if (killed) return;
  delay(1000);

  // ── Phase 2: straight 3cm then arc CW to front of obstacle ──
  mqtt.sendLog("avoid: phase 2 drive");
  driveDist(ticksForDistance(40.0f)); if (killed) return;
  mqtt.sendLog("avoid: phase 2 arc");
  encLast = micros();
  checkLast = millis();
  unsigned long arcDeadline = millis() + 1200;
  setMotors(mc, 650, 100);
  while (millis() < arcDeadline) {
    unsigned long now = micros();
    if (now - encLast >= 500) { encLast = now; pollEncoders(); }
    if (millis() - checkLast >= 5) { checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) { setMotors(mc, 0, 0); return; } }
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (millis() - lastPub >= 200) { lastPub = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  setMotors(mc, 0, 0);
  delay(1000);

  // ── Phase 3: rotate 30° CW, drive 50cm, then sweep search for IR line ──
  mqtt.sendLog("avoid: phase 3 turn");
  motion.startTurn(1, TURN_SPEED, ticksForTurn(30));
  waitForMotion(); if (killed) return;
  mqtt.sendLog("avoid: phase 3 drive");
  driveDist(ticksForDistance(50.0f)); if (killed) return;
  delay(500);
  mqtt.sendLog("avoid: phase 3 sweep");
  phaseStart = millis();
  encLast = micros();
  checkLast = millis();
  sweepToggle = millis();
  sweepDir = 1;
  bool lineFound = false;
  setMotors(mc, 400, 400);
  while (millis() - phaseStart < 20000) {
    unsigned long now = micros();
    if (now - encLast >= 500) { encLast = now; pollEncoders(); }
    if (millis() - checkLast >= 5) { checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) { setMotors(mc, 0, 0); return; } }
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (millis() - sweepToggle >= 400) { sweepToggle = millis(); sweepDir = -sweepDir; setMotors(mc, 300 + sweepDir * 200, 300 - sweepDir * 200); }
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (millis() - lastPub >= 200) { lastPub = millis(); mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }
    if (irCentroidVal >= 0) {
      mqtt.sendLog("avoid: line found");
      lineFound = true;
      break;
    }
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  if (killed) return;
  setMotors(mc, 0, 0);
  if (!lineFound) mqtt.sendLog("avoid: line not found");

  mqtt.sendLog("avoid: complete");
  state = ST_PLAN;
}

// ── Deposit Sequence ────────────────────────────────────────
void runDeposit() {
  mqtt.sendState("DEPOSIT");

  // ── 1. Move forward until RFID tag detected ───────────────
  mqtt.sendLog("deposit: searching for tag");
  float targetHeading = loc.pose.headingDeg;
  unsigned long moveStart = millis();
  bool tagFound = false;

  while (millis() - moveStart < 15000) {
    handleEStop(); if (killed) { setMotors(mc, 0, 0); return; }
    mqtt.loop();
    pollEncoders();

    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    uds.tick();
    if (imuData.ok) readIMU(imuData);
    loc.update(encL, encR, imuData.gyroZ,
               imuData.accX, imuData.accY, imuData.pitch, imuData.roll);
    if (imuData.ok) imuData.headingDeg = loc.pose.headingDeg;

    if (readRFID(rfidBuf, sizeof(rfidBuf))) {
      tagFound = true;
      setMotors(mc, 0, 0);
      break;
    }

    // Heading hold + IR line follow
    float hErr = targetHeading - imuData.headingDeg;
    if (hErr > 180.0f) hErr -= 360.0f;
    if (hErr < -180.0f) hErr += 360.0f;
    int corr = (int)(hErr * 5.0f);
    if (irCentroidVal >= 0)
      corr += (irCentroidVal - 4000) / 200;
    corr = constrain(corr, -STEERING_MAX_DIFF, STEERING_MAX_DIFF);
    setMotors(mc, MOVE_SPEED + corr, MOVE_SPEED - corr);

    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }

  if (!tagFound) {
    setMotors(mc, 0, 0);
    mqtt.sendLog("deposit: no tag found");
    return;
  }

  // ── 2. Query server for fertility ──────────────────────────
  mqtt.sendLog("deposit: checking fertility");
  lastHoleReplyRow = -1;
  lastHoleReplyCol = -1;
  mqtt.sendIsFertile(rfidBuf);
  unsigned long waitStart = millis();
  while (millis() - waitStart < 10000) {
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
    mqtt.loop();
    if (lastHoleReplyRow >= 0) break;  // got reply
    // Re-send query every 2s in case first one was dropped
    if ((millis() - waitStart) > 2000 && (millis() - waitStart) % 2000 < 25)
      mqtt.sendIsFertile(rfidBuf);
    handleEStop(); if (killed) return;
  }

  // ── 4. Use server reply coordinates ───────────────────────
  if (lastHoleReplyRow < 0) {
    mqtt.sendLog("deposit: no server reply");
    return;
  }
  uint8_t holeRow = (uint8_t)lastHoleReplyRow;
  uint8_t holeCol = (uint8_t)lastHoleReplyCol;
  if (!holeFertile[holeRow][holeCol]) {
    mqtt.sendLog("deposit: hole not fertile");
    return;
  }

  // ── 5. Move forward to align chute over hole ──────────────
  mqtt.sendLog("deposit: positioning");
  motion.startStraight(MOVE_SPEED, ticksForDistance(DEPOSIT_EXTRA_MM));
  waitForMotion(); if (killed) return;

  // ── 5. Dispense seed ───────────────────────────────────────
  mqtt.sendLog("deposit: dispensing");
  dispenseNextSeed(servo);

  // ── 6. Wiggle to clear chute ──────────────────────────────
  int wiggleSpeed = constrain(MOTOR_MIN + 30, MOTOR_MIN, MOTOR_MAX);
  motion.startStraight(wiggleSpeed, ticksForDistance(30));
  waitForMotion();
  motion.startStraight(-wiggleSpeed, ticksForDistance(30));
  waitForMotion();

  // ── 7. Mark planted ───────────────────────────────────────
  holePlanted[holeRow][holeCol] = true;
  mqtt.sendSeedPlanted(rfidBuf);
  mqtt.sendHoleStatus(holeRow, holeCol, true, holeFertile[holeRow][holeCol]);
  mqtt.sendLog("deposit: done");
}

// ── Base Exit Sequence ──────────────────────────────────────
// Navigates through the base to the exit RFID, requests exit
// permission, traverses the tunnel and exits when gravity normalises.
void runBaseExit() {
  mqtt.sendLog("base exit start");
  mqtt.sendState("EXIT_BASE");

  // ── Adaptive PD line-follow helper (blocking, scans RFID) ──
  // Returns when corner is detected or killed/timeout.
  auto followLeg = [&](int baseSpeed, float kp, float kd, int maxDiff,
                       unsigned long timeoutMs) {
    unsigned long deadline = millis() + timeoutMs;
    float prevError = 0;
    unsigned long enterMs = millis();
    unsigned long extremeMs = 0;
    bool lineLostFlagged = false;
    unsigned long _snapLast = 0;
    unsigned long _encLast = micros();
    unsigned long _checkLast = millis();

    while (millis() < deadline) {
      unsigned long _now = micros();
      if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); }
      if (millis() - _checkLast >= 5) {
        _checkLast = millis(); mqtt.loop();
        if (handleEStop()) { setMotors(mc, 0, 0); return; }
      }
      if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); return; }

      readIR(irVals);
      int centroid = irCentroid(irVals);

      if (millis() - _snapLast >= 200) { _snapLast = millis(); mqtt.sendSensorSnapshot(irVals, centroid, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal); }

      // Blind forward first 500ms — ignore centroid, just drive straight
      if (millis() - enterMs < 500) {
        setMotors(mc, baseSpeed, baseSpeed);
        { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
        continue;
      }

      // RFID scan — stop, log, resume
      if (readRFID(rfidBuf, sizeof(rfidBuf))) {
        setMotors(mc, 0, 0);
        char buf[48]; snprintf(buf, sizeof(buf), "exit: RFID tag %s", rfidBuf);
        mqtt.sendLog(buf);
        enterMs = millis();
        deadline = millis() + timeoutMs;
        extremeMs = 0;
        { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
        continue;
      }

      // Corner detection (skip first 300ms after start/RFID resume)
      if (millis() - enterMs > 300) {
        // Primary: ≥5 sensors active = perpendicular line spans array
        int active = 0;
        for (int i = 0; i < IR_COUNT; i++)
          if (irVals[i] > 400) active++;
        if (active >= 5) { setMotors(mc, 0, 0); return; }
      }

      // Line lost — sweep search for line
      if (centroid < 0) {
        if (!lineLostFlagged) { lineLostFlagged = true; mqtt.sendLog("exit: line lost"); }
        int spinDir = ((millis() - enterMs) / 1000) % 2 == 0 ? 1 : -1;
        setMotors(mc, spinDir * 400, -spinDir * 400);
        { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
        continue;
      }
      lineLostFlagged = false;

      float error = (float)centroid - 4000.0f;
      float absErr = fabsf(error);

      // Secondary: extreme error sustained > 150ms = line escaped to edge
      if (millis() - enterMs > 300) {
        if (absErr > 3500.0f) {
          if (extremeMs == 0) extremeMs = millis();
          else if (millis() - extremeMs > 150) { setMotors(mc, 0, 0); return; }
        } else { extremeMs = 0; }
      }

      int base = baseSpeed;

      // PD
      float deriv = error - prevError;
      prevError = error;
      float correction = kp * error + kd * deriv;
      if (correction > maxDiff) correction = maxDiff;
      if (correction < -maxDiff) correction = -maxDiff;

      int left = constrain(base + (int)correction, MOTOR_MIN, MOTOR_MAX);
      int right = constrain(base - (int)correction, MOTOR_MIN, MOTOR_MAX);

      setMotors(mc, left, right);

      { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
    }
    setMotors(mc, 0, 0);
  };

  const float LF_KP = 10.0f;
  const float LF_KD = 0.2f;
  const int   LF_MAX_DIFF = 250;
  const unsigned long LEG_TO = 10000;

  // ── Leg 1 ──
  mqtt.sendLog("exit leg 1");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  delay(1000);
  // motion.startStraight(250, ticksForDistance(10));
  // waitForMotion(); if (killed) { state = ST_IDLE; return; }
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90));
  mqtt.sendLog("exit turn right");
  waitForMotion(); if (killed) { state = ST_IDLE; return; }

  // ── Leg 2 ──
  mqtt.sendLog("exit leg 2");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  delay(1000);
  // motion.startStraight(250, ticksForDistance(10));
  // waitForMotion(); if (killed) { state = ST_IDLE; return; }
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90));
  mqtt.sendLog("exit turn left");
  waitForMotion(); if (killed) { state = ST_IDLE; return; }

  // ── Leg 3 ──
  mqtt.sendLog("exit leg 3");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  // ── Ask server to exit (commented out) ──
  // airlockAccepted = false;
  // mqtt.sendAirlockRequest("A", rfidBuf);
  // unsigned long _encLastA = micros();
  // unsigned long _checkLastA = millis();
  // unsigned long waitStart = millis();
  // while (millis() - waitStart < 15000) {
  //   unsigned long _nowA = micros();
  //   if (_nowA - _encLastA >= 500) { _encLastA = _nowA; pollEncoders(); }
  //   if (millis() - _checkLastA >= 5) { _checkLastA = millis(); mqtt.loop(); handleEStop(); if (killed) { state = ST_IDLE; return; } }
  //   if (airlockAccepted) break;
  //   if ((millis() - waitStart) > 2000 && (millis() - waitStart) % 2000 < 25)
  //     mqtt.sendAirlockRequest("A", rfidBuf);
  //   handleEStop(); if (killed) { state = ST_IDLE; return; }
  // }
  // if (!airlockAccepted) { mqtt.sendLog("exit: airlock denied"); state = ST_IDLE; return; }

  // ── Leg 4 ──
  mqtt.sendLog("exit leg 4");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  delay(1000);
  // motion.startStraight(250, ticksForDistance(10));
  // waitForMotion(); if (killed) { state = ST_IDLE; return; }
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90));
  mqtt.sendLog("exit turn left");
  waitForMotion(); if (killed) { state = ST_IDLE; return; }

  // ── Leg 5 ──
  mqtt.sendLog("exit leg 5");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  delay(1000);
  // motion.startStraight(250, ticksForDistance(10));
  // waitForMotion(); if (killed) { state = ST_IDLE; return; }
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90));
  mqtt.sendLog("exit turn right");
  waitForMotion(); if (killed) { state = ST_IDLE; return; }

  // ── Leg 6 ──
  mqtt.sendLog("exit leg 6");
  followLeg(500, LF_KP, LF_KD, LF_MAX_DIFF, LEG_TO);
  if (killed) { state = ST_IDLE; return; }

  // ── Tunnel traversal ──
  mqtt.sendLog("exit: tunnel 3s delay");
  { unsigned long _ts = millis(); while (millis() - _ts < 3000) { mqtt.loop(); if (handleEStop()) { state = ST_IDLE; return; } delay(5); } }

  mqtt.sendLog("exit: tunnel start");
  motion.startTunnelCentre(550, 2.0f, 80, 60000);
  {
    unsigned long _encLast = micros();
    unsigned long _checkLast = millis();
    while (motion.tick(mc, -1, filteredUdsM, filteredUdsL, filteredUdsR) == MotionSM::RUNNING) {
      unsigned long _now = micros();
      if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); }
      if (millis() - _checkLast >= 5) {
        _checkLast = millis();
        mqtt.loop();
        if (handleEStop()) { motion.stop(); setMotors(mc, 0, 0); state = ST_IDLE; return; }
        if (!mqtt.isEffectivelyEnabled()) { motion.stop(); setMotors(mc, 0, 0); state = ST_IDLE; return; }
      }
      if (filteredUdsM > 0 && filteredUdsM <= 10.0f) {
        motion.stop();
        setMotors(mc, 0, 0);
        break;
      }
    }
  }
  setMotors(mc, 0, 0);
  mqtt.sendLog("exit: at door, 2s delay");
  { unsigned long _ts = millis(); while (millis() - _ts < 2000) {
    mqtt.loop(); if (handleEStop()) { state = ST_IDLE; return; }
    if (filteredUdsM > 10.0f) break;
    delay(5);
  } }

  mqtt.sendLog("exit: tunnel exit, 200mm");
  motion.startTunnelCentre(550, 2.0f, 80, 30000);
  { long _encDoor = (abs(encL) + abs(encR)) / 2;
    long _targetTicks = ticksForDistance(200);
    unsigned long _encLast = micros();
    unsigned long _checkLast = millis();
    while (motion.tick(mc, -1, filteredUdsM, filteredUdsL, filteredUdsR) == MotionSM::RUNNING) {
      unsigned long _now = micros();
      if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); }
      if (millis() - _checkLast >= 5) {
        _checkLast = millis();
        mqtt.loop();
        if (handleEStop()) { motion.stop(); setMotors(mc, 0, 0); state = ST_IDLE; return; }
        if (!mqtt.isEffectivelyEnabled()) { motion.stop(); setMotors(mc, 0, 0); state = ST_IDLE; return; }
      }
      if ((abs(encL) + abs(encR)) / 2 - _encDoor >= _targetTicks) {
        motion.stop();
        setMotors(mc, 0, 0);
        break;
      }
    }
  }
  setMotors(mc, 0, 0);
  mqtt.sendLog("exit: tunnel done");

  state = ST_IDLE;
  mqtt.sendLog("base exit done");
}

// ── Grid Navigation + Dead Reckoning ────────────────────────
// Shared helper: drive a distance at a given heading, scanning for RFID.
// Returns true if an RFID tag was found (and snaps pose via server reply).
static bool driveSegment(float heading, long targetTicks, bool useLineFollow,
                          uint8_t& outRow, uint8_t& outCol) {
  long startEnc = (abs(encL) + abs(encR)) / 2;
  unsigned long deadline = millis() + 30000;
  while (millis() < deadline) {
    mqtt.loop();
    if (handleEStop()) { setMotors(mc, 0, 0); return false; }
    if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); Serial.println("driveSegment: disabled"); return false; }
    pollEncoders();
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    if (imuData.ok) readIMU(imuData);

    // RFID found → query server, snap position
    if (readRFID(rfidBuf, sizeof(rfidBuf))) {
      setMotors(mc, 0, 0);
      mqtt.sendIsFertile(rfidBuf);
      unsigned long t0 = millis();
      lastHoleReplyRow = -1;
      while (millis() - t0 < 5000) {
        unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } }
        mqtt.loop();
        handleEStop(); if (killed) { setMotors(mc, 0, 0); return false; }
        if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); return false; }
        if (lastHoleReplyRow >= 0) break;
      }
      if (lastHoleReplyRow >= 0) {
        outRow = (uint8_t)lastHoleReplyRow;
        outCol = (uint8_t)lastHoleReplyCol;
        loc.correctFromRFID(arena, outRow, outCol);
        return true;
      }
    }

    long avgEnc = (abs(encL) + abs(encR)) / 2;
    if (avgEnc - startEnc >= targetTicks) { setMotors(mc, 0, 0); Serial.print("driveSegment: target ticks "); Serial.println(targetTicks); return false; }

    float hErr = heading - imuData.headingDeg;
    if (hErr > 180.0f) hErr -= 360.0f;
    if (hErr < -180.0f) hErr += 360.0f;
    int corr = (int)(hErr * 5.0f);
    if (useLineFollow && irCentroidVal >= 0) {
      float lErr = (irCentroidVal - 4000) / 700.0f;
      lErr = constrain(lErr, -5.0f, 5.0f);
      corr += (int)(lErr * 5.0f);
    }
    corr = constrain(corr, -STEERING_MAX_DIFF, STEERING_MAX_DIFF);
    setMotors(mc, MOVE_SPEED + corr, MOVE_SPEED - corr);
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }
  return false;
}

static void backtrack50() {
  long target = ticksForDistance(50);
  long startL = encL, startR = encR;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    pollEncoders();
    if ((abs(encL - startL) + abs(encR - startR)) / 2 >= target) break;
    setMotors(mc, -300, -300);
    unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } }
    mqtt.loop();
    if (handleEStop()) { setMotors(mc, 0, 0); return; }
    if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); return; }
  }
  setMotors(mc, 0, 0);
  delay(200);
}

// Try forward, then +5°, then −5° from original heading.
// heading is updated to whichever heading succeeded.
static bool moveAndSnap(float distMm, float& heading, bool useLineFollow,
                         uint8_t& outRow, uint8_t& outCol) {
  float tries[] = {heading, heading + 5.0f, heading - 5.0f};
  for (int i = 0; i < 3; i++) {
    bool found = driveSegment(tries[i], ticksForDistance(distMm),
                               useLineFollow, outRow, outCol);
    heading = tries[i];
    if (found) return true;
    if (i < 2) backtrack50();
  }
  mqtt.sendLog("nav: rfid not found");
  return false;
}

static void runNodePath(bool useLineFollow) {
  mqtt.sendLog(useLineFollow ? "grid nav start" : "dea   reckon start");
  if (!useLineFollow) g_gridMults.reset();

  auto driveNode = [&]() {
    if (useLineFollow) {
      uint8_t r, c; float h = loc.pose.headingDeg;
      moveAndSnap(250.0f, h, true, r, c);
    } else {
      driveDist(ticksForDistance(HOLE_SPACING_MM));
    }
  };

  // Leg 1: forward 2 nodes
  driveNode(); mqtt.sendLog("node 1");
  driveNode(); mqtt.sendLog("node 2");
  if (killed) return;

  // Turn right 90
  motion.startTurn(1, TURN_SPEED, ticksForTurn((long)(90.0f * g_gridMults.next())));
  waitForMotion(); if (killed) return;
  delay(200);

  // Leg 2: forward 1 node
  driveNode(); mqtt.sendLog("node 3");
  if (killed) return; 

  // Turn left 90
  motion.startTurn(-1, TURN_SPEED, ticksForTurn((long)(90.0f * g_gridMults.next())));
  waitForMotion(); if (killed) return;
  delay(200);

  // Leg 3: forward 2 nodes
  driveNode(); mqtt.sendLog("node 4");
  driveNode(); mqtt.sendLog("node 5");
}

void runGridNav() {
  mqtt.sendState("GRID_NAV");
  runNodePath(true);
}

void runGridNavNoLines() {
  mqtt.sendState("GRID_NAV_NOLINES");
  runNodePath(false);
}

void runReturnBase() {
  setMotors(mc, 0, 0);
  mqtt.sendLog("returning to base");
  delay(1000);
  state = ST_IDLE;
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n===== MARTIAN ROBOTICS PROJECT =====");

  // ── MQTT (before Wire1, matches working test order) ──────
  mqtt.onEnable     = onMqttEnable;
  mqtt.onDisable    = onMqttDisable;
  mqtt.onEmergency  = onMqttEmergency;
  mqtt.onPidTune = onMqttPidTune;
  mqtt.onHoleStatus = onMqttHoleStatus;
  mqtt.onHeadingReset = onMqttHeadingReset;
  mqtt.onHeadingReset = onMqttHeadingReset;
  mqtt.onRevive       = onMqttRevive;
  mqtt.onTestCommand   = onMqttTestCommand;
  mqtt.onAirlockReply = onMqttAirlockReply;
  mqtt.onSeedSelect   = onMqttSeedSelect;
  Serial.print("MQTT: connecting");
  mqtt.begin();
  for (int i = 0; i < 50 && !mqtt.isConnected(); i++) {
    mqtt.loop();
    Serial.print(".");
    delay(100);
  }
  Serial.println(mqtt.isConnected() ? " connected" : " timeout — will retry in loop");

  // ── Encoder pins (polled in 50 Hz loop — no ISRs to avoid WiFi interference)
  pinMode(PIN_ENC_LA, INPUT_PULLUP);
  pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP);
  pinMode(PIN_ENC_RB, INPUT_PULLUP);

  // ── Motoron ──────────────────────────────────────────────
  Wire1.begin();
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.clearMotorFaultUnconditional();
  mc.setMaxAcceleration(1, MOTOR_RAMP);
  mc.setMaxDeceleration(1, MOTOR_RAMP);
  mc.setMaxAcceleration(2, MOTOR_RAMP);
  mc.setMaxDeceleration(2, MOTOR_RAMP);
  setMotors(mc, 0, 0);

  // ── Kill switch / LED ────────────────────────────────────
  pinMode(PIN_KILL_BTN, INPUT_PULLUP);
  pinMode(PIN_ACT_LED, OUTPUT);
  digitalWrite(PIN_ACT_LED, LOW);

  // ── IR sensors ───────────────────────────────────────────
  initIR();

  // ── Ultrasonic ───────────────────────────────────────────
  pinMode(PIN_UDS_LT, OUTPUT); pinMode(PIN_UDS_LE, INPUT);
  pinMode(PIN_UDS_MT, OUTPUT); pinMode(PIN_UDS_ME, INPUT);
  pinMode(PIN_UDS_RT, OUTPUT); pinMode(PIN_UDS_RE, INPUT);

  // ── Servo ────────────────────────────────────────────────
  servo.attach(PIN_SERVO);
  lockSeeds(servo);

  // ── RFID ─────────────────────────────────────────────────
  rfid.PCD_Init();

  // ── IMU ──────────────────────────────────────────────────
  initIMU(imuData);

  // ── Light Sensor ──────────────────────────────────────────
  initLightSensor();

  // ── Initial pose ─────────────────────────────────────────
  loc.setPose(0, 0, 0);

  // ── Hole fertility ───────────────────────────────────────
  for (uint8_t r = 0; r < GRID_HOLES; r++)
    for (uint8_t c = 0; c < GRID_HOLES; c++)
      holeFertile[r][c] = true;

  startTime = millis();
  Serial.println("System ready");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  mqtt.loop();

  // Serial commands for offline testing (no server)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "enable") {
      mqtt.serverAllow = true;
      mqtt.dashboardDesired = true;
      mqtt.applyState();
      Serial.println("Serial: enabled (serverAllow forced)");
    } else if (cmd == "disable") {
      mqtt.serverAllow = false;
      mqtt.dashboardDesired = false;
      mqtt.applyState();
      Serial.println("Serial: disabled");
    } else if (cmd == "kill") {
      killed = true;
      setMotors(mc, 0, 0);
      digitalWrite(PIN_ACT_LED, HIGH);
    }
  }

  static bool wasConnected = false;
  if (!wasConnected && mqtt.isConnected()) {
    Serial.println("MQTT: connected");
    wasConnected = true;
  }

  handleEStop();

  unsigned long now = millis();

  // ── 50 Hz: full sensor read (always, even when killed) ──
  if (now - lastSensorRead >= 20) {
    lastSensorRead = now;

    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    pollEncoders();
    uds.tick();
    udsL = uds.distances[UDSManager::LEFT];
    udsM = uds.distances[UDSManager::MID];
    udsR = uds.distances[UDSManager::RIGHT];
    filteredUdsL = udsLFilter.update((float)udsL);
    filteredUdsM = udsMFilter.update((float)udsM);
    filteredUdsR = udsRFilter.update((float)udsR);
    if (imuData.ok) readIMU(imuData);
    loc.update(encL, encR, imuData.gyroZ, imuData.accX, imuData.accY, imuData.pitch, imuData.roll);
    if (imuData.ok) imuData.headingDeg = loc.pose.headingDeg;
    lightVal = readLightSensor();

    // RFID: tag is 8-char opaque ID → send to server for resolution
    if (readRFID(rfidBuf, sizeof(rfidBuf))) {
      rfidSeen = true;
      mqtt.sendIsFertile(rfidBuf);
    }
  }

  // ── 5 Hz: MQTT publish (always, even when killed) ────────
  if (now - lastPublish >= 200) {
    lastPublish = now;
    mqtt.sendPose(loc.pose.x, loc.pose.y, loc.pose.headingDeg);
    if (killed) {
      mqtt.sendState("KILLED");
    } else if (!mqtt.isEffectivelyEnabled()) {
      mqtt.sendState("DISABLED");
    } else {
      mqtt.sendState(stateNames[state]);
    }
    mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR,
                            imuData.headingDeg, lightVal);
  }

  // ── When killed: stop actuators only, sensors still stream ──
  if (killed) {
    setMotors(mc, 0, 0); motion.stop();
    return;
  }

  if (!mqtt.isEffectivelyEnabled()) {
    setMotors(mc, 0, 0); motion.stop();
    return;
  }

  // ── Tick active motion (non-blocking) ────────────────────
  if (motion.type != MotionSM::IDLE) {
    int mr = motion.tick(mc, irCentroidVal, filteredUdsM, filteredUdsL, filteredUdsR);
    if (mr != MotionSM::RUNNING) {
      if (mr == MotionSM::BLOCKED) {
        mqtt.sendLog("Obstacle detected! Commencing active UDS detour.");
        motion.stop();
        state = ST_AVOID;
      }
    }
  }

  // ── Run state (only if no active motion) ─────────────────
  if (motion.type == MotionSM::IDLE) {
    switch (state) {
      case ST_INIT:        runInit();        break;
      case ST_IDLE:        runIdle();        break;
      case ST_PLAN:        runPlan();        break;
      case ST_NAVIGATE:    runNavigate();    break;
      case ST_AVOID:       runAvoid();       break;
      case ST_DEPOSIT:     runDeposit();     break;
      case ST_RETURN_BASE: runReturnBase();  break;
      case ST_EXIT_BASE:   runBaseExit();    break;
      case ST_REVIVE:      /* todo */        break;
      case ST_LET_IN:      /* todo */        break;
      case ST_TEST:        /* handled by callback */ break;
    }
  }
}
