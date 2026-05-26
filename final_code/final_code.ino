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
MQTTManager mqtt("Terminator");   // TODO: change to your robot name
MotionSM motion;                   // non-blocking motion controller

// ── Encoder ISRs ───────────────────────────────────────────
volatile long encL = 0, encR = 0;
void isr_LA() { encL += (digitalRead(PIN_ENC_LA) == digitalRead(PIN_ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(PIN_ENC_LA) != digitalRead(PIN_ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(PIN_ENC_RA) == digitalRead(PIN_ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(PIN_ENC_RA) != digitalRead(PIN_ENC_RB)) ? 1 : -1; }

// ── E-Stop ──────────────────────────────────────────────────
volatile bool killed = false;

bool handleEStop() {
  static bool lastBtn = HIGH;
  static unsigned long debounce = 0;
  bool btn = digitalRead(PIN_KILL_BTN);
  if (btn == HIGH && lastBtn == LOW && millis() - debounce > 50) {
    debounce = millis();
    killed = !killed;
    if (killed) { setMotors(mc, 0, 0); digitalWrite(PIN_ACT_LED, HIGH); }
    else        { digitalWrite(PIN_ACT_LED, LOW); }
  }
  lastBtn = btn;
  return killed;
}

void waitForUnkill() {
  while (killed) { handleEStop(); mqtt.loop(); delay(20); }
}

// ── PID tuning params (tuneable via MQTT) ───────────────────
float pidKp = 0.5, pidKi = 0.0, pidKd = 0.0;
int   pidMaxDiff = 40;

// ── State Machine ───────────────────────────────────────────
enum State {
  ST_INIT, ST_IDLE, ST_LOCATE, ST_PLAN,
  ST_NAVIGATE, ST_AVOID, ST_DEPOSIT,
  ST_RETURN_BASE, ST_REVIVE, ST_LET_IN,
  ST_TEST
};

const char* stateNames[] = {
  "INIT", "IDLE", "LOCATE", "PLAN",
  "NAVIGATE", "AVOID", "DEPOSIT",
  "RETURN_BASE", "REVIVE", "LET_IN",
  "TEST"
};

State state = ST_INIT;

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

// ── Sensor state ────────────────────────────────────────────
uint16_t irVals[IR_COUNT];
int irCentroidVal = -1;
long udsL, udsM, udsR;
char rfidBuf[32];
bool rfidSeen = false;

// ============================================================
//  MQTT CALLBACKS
// ============================================================

void onMqttEnable() {
  killed = false;
  digitalWrite(PIN_ACT_LED, LOW);
}

void onMqttDisable() {
  setMotors(mc, 0, 0);
  digitalWrite(PIN_ACT_LED, HIGH);
}

void onMqttPidTune(const String& key, float val) {
  if (key == "kp")       { pidKp = val; motion.kp = val; mqtt.sendLog("kp set"); }
  else if (key == "ki")  { pidKi = val; motion.ki = val; mqtt.sendLog("ki set"); }
  else if (key == "kd")  { pidKd = val; motion.kd = val; mqtt.sendLog("kd set"); }
  else if (key == "md")  { pidMaxDiff = constrain((int)val, 0, 130); motion.maxDiff = pidMaxDiff; mqtt.sendLog("md set"); }
}

void onMqttHoleStatus(uint8_t row, uint8_t col, bool fertile) {
  if (row < GRID_HOLES && col < GRID_HOLES)
    holeFertile[row][col] = fertile;
}

void onMqttRevive(const char* robotId) {
  mqtt.sendLog("revive requested");
}

// ── Test mode: local blocking loop with full sensor reads ───
// Blocks inside a while(1) so sensors, MQTT, and IMU all stay
// hot.  The motion controller advances one step per pass.

static void runTestLoop(unsigned long durationMs) {
  unsigned long endMs = millis() + durationMs;
  while (millis() < endMs) {
    mqtt.loop();
    if (handleEStop()) { setMotors(mc, 0, 0); motion.stop(); waitForUnkill(); break; }
    if (!mqtt.enabled) { setMotors(mc, 0, 0); motion.stop(); break; }

    // Full sensor read
    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    udsL = readUDS(PIN_UDS_LT, PIN_UDS_LE);
    udsM = readUDS(PIN_UDS_MT, PIN_UDS_ME);
    udsR = readUDS(PIN_UDS_RT, PIN_UDS_RE);
    if (imuData.ok) readIMU(imuData);
    loc.update(encL, encR, imuData.headingDeg, imuData.accX, imuData.accY);

    // Advance motion
    int mr = motion.tick(mc, irCentroidVal, (float)udsM, (float)udsL, (float)udsR);
    if (mr != MotionSM::RUNNING) break;

    // Publish every 500ms
    if (millis() - lastPublish >= 500) {
      lastPublish = millis();
      mqtt.sendPose(loc.pose.x, loc.pose.y, loc.pose.headingDeg);
      mqtt.sendState("TEST");
      mqtt.sendSensorSnapshot(irVals, irCentroidVal, udsL, udsM, udsR, imuData.headingDeg);
    }

    delay(20);
  }
  setMotors(mc, 0, 0);
  motion.stop();
}

void onMqttTestCommand(const String& cmd) {
  Serial.print("Test: "); Serial.println(cmd);

  if (cmd.startsWith("FOLLOW_LINE")) {
    int base = 500;
    float p = pidKp, i = pidKi, d = pidKd;
    int md = pidMaxDiff;
    sscanf(cmd.c_str(), "FOLLOW_LINE:%d,%f,%f,%f,%d", &base, &p, &i, &d, &md);
    motion.startLineFollow(base, p, i, d, md, 10000);
    state = ST_TEST;
    runTestLoop(10000);
    state = ST_IDLE;
    mqtt.sendLog("line follow done");
  }
  else if (cmd.startsWith("FOLLOW_WALL")) {
    int base = 500, side = 1;
    float p = 1.5, i = 0.0, d = 0.0;
    int md = pidMaxDiff;
    sscanf(cmd.c_str(), "FOLLOW_WALL:%d,%d,%f,%f,%f,%d", &base, &side, &p, &i, &d, &md);
    motion.startWallFollow(base, side, p, i, d, md, 10000);
    state = ST_TEST;
    runTestLoop(10000);
    state = ST_IDLE;
    mqtt.sendLog("wall follow done");
  }
  else if (cmd == "DEPOSIT") {
    state = ST_TEST;
    runDeposit();
    state = ST_IDLE;
    mqtt.sendLog("deposit done");
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
  state = ST_NAVIGATE;
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

  if (obstacleAhead()) { setMotors(mc, 0, 0); state = ST_AVOID; return; }

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

// ── Obstacle Avoidance (uses MotionSM non-blocking) ────────
// When entered, starts a turn → straight → turn sequence.
// The main loop ticks motion every pass.

void runAvoid() {
  static enum { TURN_RIGHT, GO, DONE } phase = TURN_RIGHT;
  static bool fresh = true;

  if (fresh) { phase = TURN_RIGHT; fresh = false; }

  if (phase == TURN_RIGHT) {
    long ticks45 = (long)(PI * TRACK_BASE_MM * 45.0 / 360.0 * TICKS_PER_M / 1000.0 / 3.0);
    if (motion.type == MotionSM::IDLE) {
      motion.startTurn(-1, 600, ticks45);
      phase = GO;
    }
  }
  else if (phase == GO) {
    if (motion.type == MotionSM::IDLE) {
      long ticks30cm = (long)(0.30 * TICKS_PER_M);
      motion.startStraight(500, ticks30cm);
      phase = DONE;
    }
  }
  else {  // DONE
    if (motion.type == MotionSM::IDLE) {
      long ticks45 = (long)(PI * TRACK_BASE_MM * 45.0 / 360.0 * TICKS_PER_M / 1000.0 / 3.0);
      motion.startTurn(1, 600, ticks45);
      phase = TURN_RIGHT;
      fresh = true;
      state = ST_PLAN;
    }
  }
}

// ── Deposit Sequence (uses MotionSM for wiggle) ────────────
void runDeposit() {
  mqtt.sendState("DEPOSIT");

  bool holeConfirmed = false;
  uint8_t holeRow = 0, holeCol = 0;

  if (readRFID(rfid, rfidBuf, sizeof(rfidBuf)))
    holeConfirmed = arena.rfidToHole(rfidBuf, holeRow, holeCol);

  if (!holeConfirmed) {
    int idx = arena.nearestHole((int16_t)loc.pose.x, (int16_t)loc.pose.y);
    if (idx >= 0) { holeRow = idx / GRID_HOLES; holeCol = idx % GRID_HOLES; holeConfirmed = true; }
  }

  if (!holeConfirmed) { mqtt.sendLog("deposit: no hole found"); return; }
  if (!holeFertile[holeRow][holeCol]) {
    mqtt.sendLog("hole not fertile");
    holePlanted[holeRow][holeCol] = true;
    return;
  }

  // Centre over hole using IR
  int baseSpeed = constrain(MOTOR_MIN + 30, MOTOR_MIN, MOTOR_MAX);
  if (irCentroidVal >= 0) {
    int left  = baseSpeed + constrain((irCentroidVal - 4000) / 100, -20, 20);
    int right = baseSpeed - constrain((irCentroidVal - 4000) / 100, -20, 20);
    left  = constrain(left,  MOTOR_MIN, MOTOR_MAX);
    right = constrain(right, MOTOR_MIN, MOTOR_MAX);
    setMotors(mc, left, right);
    delay(300);
    setMotors(mc, 0, 0);
  }

  // Dispense seed
  mqtt.sendLog("dispensing seed");
  dispenseSeed(servo, 1);
  delay(300);

  // Wiggle forward/backward to clear chute (non-blocking via motion)
  motion.startStraight(baseSpeed, (long)(0.03 * TICKS_PER_M));
  while (motion.tick(mc) == MotionSM::RUNNING) {
    mqtt.loop(); delay(20);
  }
  motion.startStraight(-baseSpeed, (long)(0.03 * TICKS_PER_M));
  while (motion.tick(mc) == MotionSM::RUNNING) {
    mqtt.loop(); delay(20);
  }

  lockSeeds(servo);

  holePlanted[holeRow][holeCol] = true;
  mqtt.sendHoleStatus(holeRow, holeCol, true, holeFertile[holeRow][holeCol]);
  mqtt.sendLog("seed planted");
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
  Serial.begin(9600);
  delay(1500);
  Serial.println("\n===== MARTIAN ROBOTICS PROJECT =====");

  // ── Encoder ISRs ─────────────────────────────────────────
  pinMode(PIN_ENC_LA, INPUT_PULLUP);
  pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP);
  pinMode(PIN_ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), isr_LA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LB), isr_LB, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), isr_RA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RB), isr_RB, RISING);

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

  // ── MQTT ─────────────────────────────────────────────────
  mqtt.onEnable     = onMqttEnable;
  mqtt.onDisable    = onMqttDisable;
  mqtt.onPidTune    = onMqttPidTune;
  mqtt.onHoleStatus = onMqttHoleStatus;
  mqtt.onRevive     = onMqttRevive;
  mqtt.onTestCommand = onMqttTestCommand;
  mqtt.begin();

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
  handleEStop();

  if (killed) {
    setMotors(mc, 0, 0); motion.stop();
    digitalWrite(PIN_ACT_LED, HIGH);
    waitForUnkill();
    digitalWrite(PIN_ACT_LED, LOW);
    return;
  }

  if (!mqtt.enabled) {
    setMotors(mc, 0, 0); motion.stop();
    delay(20);
    return;
  }

  unsigned long now = millis();

  // ── 50 Hz: full sensor read ──────────────────────────────
  if (now - lastSensorRead >= 20) {
    lastSensorRead = now;

    readIR(irVals);
    irCentroidVal = irCentroid(irVals);
    udsL = readUDS(PIN_UDS_LT, PIN_UDS_LE);
    udsM = readUDS(PIN_UDS_MT, PIN_UDS_ME);
    udsR = readUDS(PIN_UDS_RT, PIN_UDS_RE);
    if (imuData.ok) readIMU(imuData);
    loc.update(encL, encR, imuData.headingDeg, imuData.accX, imuData.accY);

    // RFID check
    if (readRFID(rfid, rfidBuf, sizeof(rfidBuf))) {
      rfidSeen = true;
      uint8_t r, c;
      if (arena.rfidToHole(rfidBuf, r, c)) {
        loc.correctFromRFID(arena, r, c);
        mqtt.sendLog("RFID correction");
      }
    }
  }

  // ── 2 Hz: MQTT publish ───────────────────────────────────
  if (now - lastPublish >= 500) {
    lastPublish = now;
    mqtt.sendPose(loc.pose.x, loc.pose.y, loc.pose.headingDeg);
    mqtt.sendState(stateNames[state]);
    mqtt.sendSensorSnapshot(irVals, irCentroidVal, udsL, udsM, udsR,
                            imuData.headingDeg);
  }

  // ── Tick active motion (non-blocking) ────────────────────
  if (motion.type != MotionSM::IDLE) {
    int mr = motion.tick(mc, irCentroidVal, (float)udsM, (float)udsL, (float)udsR);
    if (mr != MotionSM::RUNNING) {
      if (mr == MotionSM::BLOCKED) {
        mqtt.sendLog("motion blocked");
        motion.stop();
      }
      // Motion done — keep current state; the next switch case
      // below will run if state expects motion completion.
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
      case ST_REVIVE:      /* todo */        break;
      case ST_LET_IN:      /* todo */        break;
      case ST_TEST:        /* handled by callback */ break;
    }
  }

  delay(5);
}
