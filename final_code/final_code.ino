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
TurnMultSet g_avoidMults, g_gridMults, g_exitTurnMults, g_exitMoveMults;

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
  else if (cmd.startsWith("OVERRIDE_AVOID_TURNS:")) {
    g_avoidMults.parse(cmd.c_str() + 21);
    char lb[64]; snprintf(lb, sizeof(lb), "avoid mults: %d values", g_avoidMults.count);
    mqtt.sendLog(lb);
  }
  else if (cmd.startsWith("OVERRIDE_GRID_TURNS:")) {
    g_gridMults.parse(cmd.c_str() + 20);
    char lb[64]; snprintf(lb, sizeof(lb), "grid mults: %d values", g_gridMults.count);
    mqtt.sendLog(lb);
  }
  else if (cmd.startsWith("OVERRIDE_EXIT_TURNS:")) {
    g_exitTurnMults.parse(cmd.c_str() + 20);
    char lb[64]; snprintf(lb, sizeof(lb), "exit turn mults: %d values", g_exitTurnMults.count);
    mqtt.sendLog(lb);
  }
  else if (cmd.startsWith("OVERRIDE_EXIT_MOVES:")) {
    g_exitMoveMults.parse(cmd.c_str() + 20);
    char lb[64]; snprintf(lb, sizeof(lb), "exit move mults: %d values", g_exitMoveMults.count);
    mqtt.sendLog(lb);
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
  long sL = encL, sR = encR;
  long rfidStart = ticksForDistance(70);
  bool canRFID = false;
  while (true) {
    unsigned long _n = micros();
    if (_n - _eLast >= 500) { _eLast = _n; pollEncoders(); }
    if ((long)(millis() - _mDead) >= 0) { mqtt.loop(); _mDead = millis() + 5; }
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
// Boxes around: turn right, forward 1, turn left, forward 3,
// turn left, forward 1, turn right, forward 1.

void runAvoid() {
  g_avoidMults.reset();

  // Move 1 tile forward, then box around the obstacle
  mqtt.sendLog("avoid: forward 1");
  driveDist(ticksForDistance(HOLE_SPACING_MM)); if (killed) return;

  mqtt.sendLog("avoid: turn right");
  motion.startTurn(1, TURN_SPEED, ticksForTurn((long)(90.0f * g_avoidMults.next())));
  waitForMotion(); if (killed) return;

  mqtt.sendLog("avoid: forward 1");
  driveDist(ticksForDistance(HOLE_SPACING_MM)); if (killed) return;

  mqtt.sendLog("avoid: turn left");
  motion.startTurn(-1, TURN_SPEED, ticksForTurn((long)(90.0f * g_avoidMults.next())));
  waitForMotion(); if (killed) return;

  mqtt.sendLog("avoid: forward 2");
  driveDist(ticksForDistance(HOLE_SPACING_MM * 2)); if (killed) return;

  mqtt.sendLog("avoid: turn left");
  motion.startTurn(-1, TURN_SPEED, ticksForTurn((long)(90.0f * g_avoidMults.next())));
  waitForMotion(); if (killed) return;

  mqtt.sendLog("avoid: forward 1");
  driveDist(ticksForDistance(HOLE_SPACING_MM)); if (killed) return;

  mqtt.sendLog("avoid: turn right");
  motion.startTurn(1, TURN_SPEED, ticksForTurn((long)(90.0f * g_avoidMults.next())));
  waitForMotion(); if (killed) return;

  mqtt.sendLog("avoid: forward 1");
  driveDist(ticksForDistance(HOLE_SPACING_MM)); if (killed) return;

  mqtt.sendLog("avoid: detour complete");
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
  g_exitTurnMults.reset();
  g_exitMoveMults.reset();

  // Component Y-positions (mm from centre, +Y forward)
  const float CHASSIS_BACK_Y  = -CHASSIS_LENGTH / 2;
  const float CHASSIS_FRONT_Y =  CHASSIS_LENGTH / 2;
  const float IR_TO_BACK_MM  = POS_IR_CENTRE_Y - CHASSIS_BACK_Y;
  const float IR_TO_FRONT_MM = CHASSIS_FRONT_Y - POS_IR_CENTRE_Y;
  const float IR_TO_RFID_MM  = POS_IR_CENTRE_Y - POS_RFID_ANTENNA_Y;
  const float CHASSIS_BACK_TO_TREAD_BACK = 10;

  const float EXIT_LEG1_MM = 460.0f - IR_TO_BACK_MM - CHASSIS_BACK_TO_TREAD_BACK;    // 460 - 102 = 358mm
  const float EXIT_LEG2_MM = 330.0f;                     // 330mm
  const float EXIT_LEG3_MM = 205.0f - IR_TO_RFID_MM;    // 205 - 27 = 178mm
  const float EXIT_LEG4_MM = 205.0f + IR_TO_RFID_MM;    // 205 + 27 = 232mm
  const float EXIT_LEG5_MM = 330.0f;                     // 330mm
  const float EXIT_LEG6_MM = 320.0f - IR_TO_FRONT_MM;   // 320 - 68 = 252mm
  const int   EXIT_TUNNEL_SPEED = 660;


  // ── Leg 1: forward ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG1_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 1");
  waitForMotion(); if (killed) return;

  // ── Turn right 90 ──
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90.0f * g_exitTurnMults.next()));
  mqtt.sendLog("exit turn right");
  waitForMotion(); if (killed) return;

  // ── Leg 2: forward ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG2_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 2");
  waitForMotion(); if (killed) return;

  // ── Turn left 90 ──
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90.0f * g_exitTurnMults.next()));
  mqtt.sendLog("exit turn left");
  waitForMotion(); if (killed) return;

  // ── Leg 3: forward ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG3_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 3");
  waitForMotion(); if (killed) return;

  // ── Scan RFID ──
  mqtt.sendLog("exit: scanning RFID");
  bool tagFound = false;
  for (int attempt = 0; attempt < 5 && !tagFound; attempt++) {
    motion.startStraight(300, ticksForDistance(20));
    unsigned long _encLast = micros();
    unsigned long _checkLast = millis();
    while (motion.tick(mc) == MotionSM::RUNNING) {
      unsigned long _now = micros();
      if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); }
      if (millis() - _checkLast >= 5) { _checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) return; }
      if (readRFID(rfidBuf, sizeof(rfidBuf))) { setMotors(mc, 0, 0); motion.stop(); tagFound = true; break; }
    }
  }
  if (!tagFound) {
    for (int attempt = 0; attempt < 5 && !tagFound; attempt++) {
      motion.startStraight(-300, ticksForDistance(20));
      unsigned long _encLast = micros();
      unsigned long _checkLast = millis();
      while (motion.tick(mc) == MotionSM::RUNNING) {
        unsigned long _now = micros();
        if (_now - _encLast >= 500) { _encLast = _now; pollEncoders(); }
        if (millis() - _checkLast >= 5) { _checkLast = millis(); mqtt.loop(); handleEStop(); if (killed) return; }
        if (readRFID(rfidBuf, sizeof(rfidBuf))) { setMotors(mc, 0, 0); motion.stop(); tagFound = true; break; }
      }
    }
  }
  if (!tagFound) { mqtt.sendLog("exit: no RFID"); return; }
  mqtt.sendLog("exit: RFID found");

  // ── Ask server to exit ──
  airlockAccepted = false;
  mqtt.sendAirlockRequest("A", rfidBuf);
  unsigned long _encLastA = micros();
  unsigned long _checkLastA = millis();
  unsigned long waitStart = millis();
  while (millis() - waitStart < 15000) {
    unsigned long _nowA = micros();
    if (_nowA - _encLastA >= 500) { _encLastA = _nowA; pollEncoders(); }
    if (millis() - _checkLastA >= 5) { _checkLastA = millis(); mqtt.loop(); handleEStop(); if (killed) return; }
    if (airlockAccepted) break;
    if ((millis() - waitStart) > 2000 && (millis() - waitStart) % 2000 < 25)
      mqtt.sendAirlockRequest("A", rfidBuf);
    handleEStop(); if (killed) return;
  }
  if (!airlockAccepted) { mqtt.sendLog("exit: airlock denied"); return; }

  // ── Leg 4: forward ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG4_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 4");
  waitForMotion(); if (killed) return;

  // ── Turn left 90 ──
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90.0f * g_exitTurnMults.next()));
  mqtt.sendLog("exit turn left");
  waitForMotion(); if (killed) return;

  // ── Leg 5: forward ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG5_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 5");
  waitForMotion(); if (killed) return;

  // ── Turn right 90 ──
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90.0f * g_exitTurnMults.next()));
  mqtt.sendLog("exit turn right");
  waitForMotion(); if (killed) return;

  // ── Leg 6: forward (to tunnel entrance) ──
  motion.startStraight(MOVE_SPEED, ticksForDistance(EXIT_LEG6_MM * g_exitMoveMults.next()));
  mqtt.sendLog("exit leg 6");
  waitForMotion(); if (killed) return;

  // // ── Wait for tunnel door to open ──
  // mqtt.sendLog("exit: waiting for tunnel door");
  // delay(500);  // settle
  // // First wait for door to close (if it's already open)
  unsigned long doorWait = millis();
  // bool doorClosed = false;
  // while (millis() - doorWait < 10000) {
  //   mqtt.loop(); #20); handleEStop(); if (killed) return;
  //   uds.tick();
  //   filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
  //   if (filteredUdsM < 30.0f) { doorClosed = true; break; }
  // }
  // if (!doorClosed) { mqtt.sendLog("exit: tunnel already open, proceeding"); }
  // else {
  //   // Door is closed, wait for it to open
  //   while (true) {
  //     mqtt.loop(); delay(20); handleEStop(); if (killed) return;
  //     uds.tick();
  //     filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
  //     if (filteredUdsM > 50.0f) break;
  //   }
  // }
  delay(3000);

  // ── Enter tunnel: CENTRE_TUNNEL at max speed ──
  // Capture baseline pitch before slope
  float exitPitchRef = imuData.pitch;
  bool pitchChanged = false;
  mqtt.sendLog("exit: entering tunnel");
  motion.startTunnelCentre(EXIT_TUNNEL_SPEED, pidKp, pidMaxDiff);
  while (true) {
    mqtt.loop();
    if (handleEStop()) { motion.stop(); setMotors(mc, 0, 0); waitForUnkill(); return; }
    pollEncoders();
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (imuData.ok) readIMU(imuData);

    int mr = motion.tick(mc, -1, filteredUdsM, filteredUdsL, filteredUdsR);
    if (mr != MotionSM::RUNNING) break;

    // Check front UDS for second door
    if (filteredUdsM < 10.0f) { motion.stop(); setMotors(mc, 0, 0); break; }
    { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  }

  // ── Wait for second door to open ──
  mqtt.sendLog("exit: waiting for second door");
  doorWait = millis();
  { unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } } }
  while (millis() - doorWait < 15000) {
    unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } }
    mqtt.loop(); handleEStop(); if (killed) return;
    uds.tick();
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    if (filteredUdsM > 17.0f) break;
  }

  // ── Continue forward until gravity normalises ──
  mqtt.sendLog("exit: continuing past tunnel");
  setMotors(mc, EXIT_TUNNEL_SPEED, EXIT_TUNNEL_SPEED);
  unsigned long pitchStart = millis();
  while (millis() - pitchStart < 30000) {
    unsigned long _encDeadline = micros() + 20000; unsigned long _encLastE = micros(); while (micros() < _encDeadline) { unsigned long _nowE = micros(); if (_nowE - _encLastE >= 500) { _encLastE = _nowE; pollEncoders(); } }
    mqtt.loop();
    if (handleEStop()) { setMotors(mc, 0, 0); waitForUnkill(); return; }
    if (imuData.ok) readIMU(imuData);
    if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); return; }

    float pitchDiff = fabsf(imuData.pitch - exitPitchRef);
    if (pitchDiff > 8.0f) pitchChanged = true;
    if (pitchChanged && pitchDiff < 3.0f) break;
  }
  setMotors(mc, 0, 0);
  mqtt.sendLog("base exit done");
}

// ── Base Exit — Line Follow (creep + correct) ─────────────────
// Moves 10cm at a time, reads IR centroid, corrects heading.
// Detects T-junctions and turns (left/right) via IR activation patterns.
// Path: start → T (right) → left turn → RFID → left turn → right turn → door.

void runBaseExitLineFollow() {
  mqtt.sendLog("base exit lf start");
  mqtt.sendState("EXIT_BASE");

  const int   CREEP_SPEED     = 400;
  const long  STEP_TICKS      = ticksForDistance(30.0f);
  const int   CORR_SPEED      = 300;
  const float DRIFT_GAIN      = 10.0f;   // deg per unit drift (heading error)
  const float POS_GAIN        = 3.0f;    // deg per unit offset (position error)
  const int   IR_THRESH       = 400;
  const int   T_SENSOR_CNT    = 8;    // at least 8 of 9 sensors
  const int   TURN_SENSOR_CNT = 5;
  int         intCount        = 0;    // persistence: same type on 2 calls

  // Per-sensor EMA on filtered IR values
  EMA         irEMAs[IR_COUNT];
  float       irFilt[IR_COUNT];

  auto readFilteredIR = [&]() -> int {
    readIR(irVals);
    int sum = 0, weighted = 0;
    for (int i = 0; i < IR_COUNT; i++) {
      irFilt[i] = irEMAs[i].update((float)irVals[i]);
      if (irFilt[i] > 50) { sum += (int)irFilt[i]; weighted += (int)irFilt[i] * i * 1000; }
    }
    if (sum < 100) return -1;
    return weighted / sum;
  };

  // Intersection types: 0=none, -1=left, 1=right, 2=T
  auto intersectionType = [&]() -> int {
    readFilteredIR();
    int l = 0, c = 0, r = 0;
    for (int i = 0; i < 3; i++) if (irFilt[i] > IR_THRESH) l++;
    for (int i = 3; i < 6; i++) if (irFilt[i] > IR_THRESH) c++;
    for (int i = 6; i < 9; i++) if (irFilt[i] > IR_THRESH) r++;
    int t = l + c + r;
    int type = 0;
    if (t >= T_SENSOR_CNT && l >= 2 && c >= 2 && r >= 2) type = 2;
    else if (t >= TURN_SENSOR_CNT && l >= 2 && r < 2)    type = -1;
    else if (t >= TURN_SENSOR_CNT && r >= 2 && l < 2)    type = 1;
    // Require same type on 2 consecutive calls to trigger
    if (type != 0 && type == intCount)  { intCount = 0; return type; }
    if (type != 0)                      { intCount = type; return 0; }
    intCount = 0;
    return 0;
  };

  // Creep forward one step: read centroid before/after to get drift,
  // then turn on the spot to correct heading + a bit of position error.
  auto creepStep = [&]() -> bool {
    int cBefore = readFilteredIR();

    motion.startStraight(CREEP_SPEED, STEP_TICKS);
    waitForMotion(); if (killed) return false;

    delay(80);  // settle so robot is fully stopped before turning

    int cAfter = readFilteredIR();

    if (cAfter >= 0) {
      int drift  = (cBefore >= 0) ? (cAfter - cBefore) : 0;
      int posErr = cAfter - 4000;

      float weighted = (float)drift / 4000.0f * DRIFT_GAIN
                     + (float)posErr / 4000.0f * POS_GAIN;
      float ang = fabs(weighted);
      if (ang >= 2.0f) {
        motion.startTurn(weighted > 0 ? 1 : -1, CORR_SPEED, ticksForTurn(ang));
        waitForMotion(); if (killed) return false;
      }
    }

    // Publish sensor snapshot so dashboard stays live during blocking
    irCentroidVal = cAfter;
    for (int i = 0; i < IR_COUNT; i++) irVals[i] = (uint16_t)irFilt[i];
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (imuData.ok) readIMU(imuData);
    mqtt.sendSensorSnapshot(irVals, irCentroidVal, (long)filteredUdsL, (long)filteredUdsM, (long)filteredUdsR, imuData.headingDeg, lightVal);

    return true;
  };

  // ── Leg 1: Forward to T-junction ──
  mqtt.sendLog("lf leg1 T");
  while (true) {
    if (!creepStep()) return;
    if (intersectionType() == 2) break;
  }
  mqtt.sendLog("lf leg1 turn R");
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90));
  waitForMotion(); if (killed) return;

  // ── Leg 2: Forward to left turn ──
  mqtt.sendLog("lf leg2 left");
  while (true) {
    if (!creepStep()) return;
    if (intersectionType() == -1) break;
  }
  mqtt.sendLog("lf leg2 turn L");
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90));
  waitForMotion(); if (killed) return;

  // ── Leg 3: Forward scanning RFID ──
  mqtt.sendLog("lf leg3 RFID");
  {
    bool tagFound = false;
    for (int step = 0; step < 15 && !tagFound; step++) {
      if (!creepStep()) return;
      if (readRFID(rfidBuf, sizeof(rfidBuf))) { tagFound = true; break; }
    }
    // Try reversing if not found
    for (int step = 0; step < 8 && !tagFound; step++) {
      motion.startStraight(-CREEP_SPEED, STEP_TICKS);
      waitForMotion(); if (killed) return;
      if (readRFID(rfidBuf, sizeof(rfidBuf))) { tagFound = true; break; }
    }
    if (!tagFound) { mqtt.sendLog("lf leg3 no RFID"); return; }
    mqtt.sendLog("lf leg3 RFID found");
  }

  // ── Airlock request ──
  mqtt.sendLog("lf airlock request");
  airlockAccepted = false;
  mqtt.sendAirlockRequest("A", rfidBuf);
  {
    unsigned long _e = micros(), _c = millis(), _w = millis();
    while (millis() - _w < 15000) {
      if (micros() - _e >= 500) { _e = micros(); pollEncoders(); }
      if ((long)(millis() - _c) >= 5) { _c = millis(); mqtt.loop(); handleEStop(); if (killed) return; }
      if (airlockAccepted) break;
      if ((millis() - _w) > 2000 && (millis() - _w) % 2000 < 25)
        mqtt.sendAirlockRequest("A", rfidBuf);
    }
  }
  if (!airlockAccepted) { mqtt.sendLog("lf airlock denied"); return; }

  // ── Leg 4: Forward to left turn ──
  mqtt.sendLog("lf leg4 left");
  while (true) {
    if (!creepStep()) return;
    if (intersectionType() == -1) break;
  }
  mqtt.sendLog("lf leg4 turn L");
  motion.startTurn(-1, TURN_SPEED, ticksForTurn(90));
  waitForMotion(); if (killed) return;

  // ── Leg 5: Forward to right turn ──
  mqtt.sendLog("lf leg5 right");
  while (true) {
    if (!creepStep()) return;
    if (intersectionType() == 1) break;
  }
  mqtt.sendLog("lf leg5 turn R");
  motion.startTurn(1, TURN_SPEED, ticksForTurn(90));
  waitForMotion(); if (killed) return;

  // ── Leg 6: Forward to tunnel door ──
  mqtt.sendLog("lf leg6 to door");
  for (int i = 0; i < 5; i++) {
    if (!creepStep()) return;
  }

  // ── Wait for tunnel door ──
  delay(3000);

  // ── Enter tunnel: CENTRE_TUNNEL (same as original) ──
  float exitPitchRef = imuData.pitch;
  bool pitchChanged = false;
  mqtt.sendLog("lf entering tunnel");
  motion.startTunnelCentre(660, pidKp, pidMaxDiff);
  while (true) {
    mqtt.loop();
    if (handleEStop()) { motion.stop(); setMotors(mc, 0, 0); waitForUnkill(); return; }
    pollEncoders();
    uds.tick();
    filteredUdsL = udsLFilter.update((float)uds.distances[UDSManager::LEFT]);
    filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
    filteredUdsR = udsRFilter.update((float)uds.distances[UDSManager::RIGHT]);
    if (imuData.ok) readIMU(imuData);

    int mr = motion.tick(mc, -1, filteredUdsM, filteredUdsL, filteredUdsR);
    if (mr != MotionSM::RUNNING) break;

    if (filteredUdsM < 10.0f) { motion.stop(); setMotors(mc, 0, 0); break; }
    { unsigned long _d = micros() + 20000, _e = micros(); while (micros() < _d) { if (micros() - _e >= 500) { _e = micros(); pollEncoders(); } } }
  }

  // ── Wait for second door to open ──
  mqtt.sendLog("lf waiting for second door");
  {
    unsigned long _dw = millis();
    { unsigned long _d = micros() + 20000, _e = micros(); while (micros() < _d) { if (micros() - _e >= 500) { _e = micros(); pollEncoders(); } } }
    while (millis() - _dw < 15000) {
      { unsigned long _d = micros() + 20000, _e = micros(); while (micros() < _d) { if (micros() - _e >= 500) { _e = micros(); pollEncoders(); } } }
      mqtt.loop(); handleEStop(); if (killed) return;
      uds.tick();
      filteredUdsM = udsMFilter.update((float)uds.distances[UDSManager::MID]);
      if (filteredUdsM > 17.0f) break;
    }
  }

  // ── Continue until gravity normalises ──
  mqtt.sendLog("lf continuing past tunnel");
  setMotors(mc, 660, 660);
  {
    unsigned long _ps = millis();
    while (millis() - _ps < 30000) {
      { unsigned long _d = micros() + 20000, _e = micros(); while (micros() < _d) { if (micros() - _e >= 500) { _e = micros(); pollEncoders(); } } }
      mqtt.loop();
      if (handleEStop()) { setMotors(mc, 0, 0); waitForUnkill(); return; }
      if (imuData.ok) readIMU(imuData);
      if (!mqtt.isEffectivelyEnabled()) { setMotors(mc, 0, 0); return; }
      float pd = fabsf(imuData.pitch - exitPitchRef);
      if (pd > 8.0f) pitchChanged = true;
      if (pitchChanged && pd < 3.0f) break;
    }
  }
  setMotors(mc, 0, 0);
  mqtt.sendLog("base exit lf done");
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
