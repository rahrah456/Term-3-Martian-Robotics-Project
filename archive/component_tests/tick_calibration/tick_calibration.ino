// ============================================================
//  TICK CALIBRATION  —  forward / turn by distance or angle
//  Open-loop (demo 9 pattern).  2kHz encoder polling.
//
//  Commands (send via Serial Monitor, 115200 baud):
//    d <mm>             — go forward <mm> (uses MOVE_SPEED=500)
//    a <degrees>        — tank turn <degrees> (uses TURN_SPEED=550)
//    f <ticks> <speed>  — forward by raw ticks (for verification)
//    t <ticks> <speed>  — tank turn by raw ticks
//    s                  — stop
//    r                  — reset encoders
//    diag               — encoder diagnostics (5s)
//
//  After each d / a command the sketch prints the actual ticks
//  travelled.  Record (input, actualTicks) pairs in a
//  spreadsheet, fit a line, and update DIST_TICK_M / _C in
//  Config.h (for distance, mm) and TURN_TICK_M / _C (for turn, deg).
// ============================================================

#include <Arduino.h>
#include <Motoron.h>

// ── Pins ────────────────────────────────────────────────────
const int ENC_LA = 4, ENC_LB = 5;    // left track
const int ENC_RA = 2, ENC_RB = 3;    // right track
const int KILL_BTN = 38;
const int ACT_LED = 39;

// ── Calibration (mirrors Config.h — update after regression) ─
const float TICKS_PER_M = 3607.0;
const long  TICKS_PER_90 = 525;
const int   MOVE_SPEED = 500;
const int   TURN_SPEED = 550;
const float BIAS_RIGHT = 1.042f;

static inline long ticksForDistance(float mm) {
  return (long)((TICKS_PER_M / 1000.0f) * mm);
}
static inline long ticksForTurn(float deg) {
  return (long)((float)TICKS_PER_90 / 90.0f * deg);
}

// ── Motoron ─────────────────────────────────────────────────
MotoronI2C mc(0x10);

void setMotors(int left, int right) {
  right = (int)(right * BIAS_RIGHT);
  left  = constrain(left,  -660, 660);
  right = constrain(right, -660, 660);
  mc.setSpeed(2, -left);    // M2 = left track, polarity reversed
  mc.setSpeed(1, right);    // M1 = right track
}

// ── Encoders (polled at 2kHz) ───────────────────────────────
long encL = 0, encR = 0;

static const int8_t QUAD_TABLE[16] = {
  0,  1, -1,  0,
  -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

void pollEncoders() {
  static int prevL = 0, prevR = 0;
  int aL = digitalRead(ENC_LA);
  int bL = digitalRead(ENC_LB);
  int aR = digitalRead(ENC_RA);
  int bR = digitalRead(ENC_RB);
  int stateL = (aL << 1) | bL;
  int stateR = (aR << 1) | bR;
  encL += QUAD_TABLE[(prevL << 2) | stateL];
  encR += QUAD_TABLE[(prevR << 2) | stateR];
  prevL = stateL;
  prevR = stateR;
}

// ── E-Stop ──────────────────────────────────────────────────
bool killed = false;

bool handleEStop() {
  static bool lastBtn = HIGH;
  static unsigned long debounce = 0;
  bool btn = digitalRead(KILL_BTN);
  if (btn == HIGH && lastBtn == LOW && millis() - debounce > 50) {
    debounce = millis();
    killed = !killed;
    if (killed) { setMotors(0, 0); digitalWrite(ACT_LED, HIGH); }
    else        { digitalWrite(ACT_LED, LOW); }
  }
  lastBtn = btn;
  return killed;
}

// ── Blocking open-loop motion ──────────────────────────────
void runOL(int left, int right, long targetTicks, const char* label) {
  encL = 0; encR = 0;
  setMotors(left, right);

  unsigned long lastPoll = 0;
  unsigned long lastPrint = 0;

  while (true) {
    unsigned long now = micros();
    if (now - lastPoll >= 500) { lastPoll = now; pollEncoders(); }
    if (now / 1000 - lastPrint >= 1000) {
      lastPrint = now / 1000;
      Serial.print(encL); Serial.print(" "); Serial.println(encR);
    }
    handleEStop();
    if (killed) { setMotors(0, 0); return; }
    if (abs(encL) >= targetTicks && abs(encR) >= targetTicks) break;
  }
  setMotors(0, 0);

  long actualL = abs(encL);
  long actualR = abs(encR);
  long actual  = max(actualL, actualR);
  Serial.print(label);
  Serial.print("  L:"); Serial.print(actualL);
  Serial.print("  R:"); Serial.print(actualR);
  Serial.print("  target="); Serial.println(targetTicks);
  Serial.print("RECORD: "); Serial.print(actual); Serial.println(" ticks");
}

// ── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Tick Calibration (open-loop, 2kHz polling) ===");
  Serial.println("  d <mm>             forward <mm>  (MOVE_SPEED=500)");
  Serial.println("  a <degrees>        tank turn     (TURN_SPEED=550)");
  Serial.println("  f <ticks> <speed>  forward raw ticks");
  Serial.println("  t <ticks> <speed>  turn raw ticks");
  Serial.println("  s                  stop");
  Serial.println("  r                  reset encoders");
  Serial.println("  diag               encoder diagnostics");
  Serial.println();
  Serial.println("Record the 'RECORD: <ticks>' values and fit a line:");
  Serial.println("  distance: ticks = m * mm + c  →  update DIST_TICK_M/_C");
  Serial.println("  turn:     ticks = m * deg + c →  update TURN_TICK_M/_C");

  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  pinMode(KILL_BTN, INPUT_PULLUP);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);

  Wire1.begin();
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();
  mc.clearMotorFaultUnconditional();
  mc.setMaxAcceleration(1, 800);
  mc.setMaxDeceleration(1, 800);
  mc.setMaxAcceleration(2, 800);
  mc.setMaxDeceleration(2, 800);
  setMotors(0, 0);

  Serial.println("Ready\n");
}

// ── Loop ────────────────────────────────────────────────────
void loop() {
  handleEStop();

  static unsigned long lastPoll = 0;
  unsigned long now = micros();
  if (now - lastPoll >= 500) { lastPoll = now; pollEncoders(); }

  // Print encoder ticks every 1s
  static unsigned long lastPrint = 0;
  if (now / 1000 - lastPrint >= 1000) {
    lastPrint = now / 1000;
    Serial.print(encL); Serial.print(" "); Serial.println(encR);
  }

  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line[0];
  int space = line.indexOf(' ');
  String arg = (space > 0) ? line.substring(space + 1) : "";
  float val = arg.toFloat();

  if (cmd == 's') { setMotors(0, 0); Serial.println("STOP"); }
  else if (cmd == 'r') { encL = 0; encR = 0; Serial.println("encoders reset"); }

  else if (cmd == 'd' && val > 0) {
    long t = ticksForDistance(val);
    char buf[64]; snprintf(buf, sizeof(buf), "forward %.0fmm", val);
    Serial.print("target="); Serial.print(val); Serial.print("mm  cmd_ticks="); Serial.println(t);
    runOL(MOVE_SPEED, MOVE_SPEED, t, buf);
    Serial.println("DONE");
  }
  else if (cmd == 'a' && val > 0) {
    long t = ticksForTurn(val);
    char buf[64]; snprintf(buf, sizeof(buf), "turn %.0fdeg", val);
    Serial.print("target="); Serial.print(val); Serial.print("deg  cmd_ticks="); Serial.println(t);
    runOL(-TURN_SPEED, TURN_SPEED, t, buf);
    Serial.println("DONE");
  }

  else if (cmd == 'f') {
    long ticks = (long)val;
    int speed = MOVE_SPEED;
    int secondSpace = arg.indexOf(' ');
    if (secondSpace > 0) { speed = constrain((int)arg.substring(secondSpace + 1).toFloat(), 0, 660); ticks = (long)arg.substring(0, secondSpace).toFloat(); }
    char buf[64]; snprintf(buf, sizeof(buf), "forward %ld @ %d", ticks, speed);
    runOL(speed, speed, ticks, buf);
    Serial.println("DONE");
  }
  else if (cmd == 't') {
    long ticks = (long)val;
    int speed = TURN_SPEED;
    int secondSpace = arg.indexOf(' ');
    if (secondSpace > 0) { speed = constrain((int)arg.substring(secondSpace + 1).toFloat(), 0, 660); ticks = (long)arg.substring(0, secondSpace).toFloat(); }
    char buf[64]; snprintf(buf, sizeof(buf), "turn %ld @ %d", ticks, speed);
    runOL(-speed, speed, ticks, buf);
    Serial.println("DONE");
  }
  else if (line == "diag") {
    Serial.println("Encoder diagnostics (5s, spin each wheel manually):");
    for (int i = 0; i < 50; i++) {
      for (int j = 0; j < 20; j++) { pollEncoders(); delayMicroseconds(500); }
      Serial.print("L:"); Serial.print(digitalRead(ENC_LA)); Serial.print(digitalRead(ENC_LB));
      Serial.print("  R:"); Serial.print(digitalRead(ENC_RA)); Serial.print(digitalRead(ENC_RB));
      Serial.print("  cnt:"); Serial.print(encL); Serial.print(","); Serial.println(encR);
      delay(90);
    }
  }
  else {
    Serial.println("?  use: d <cm> | a <deg> | f <ticks> <speed> | t <ticks> <speed> | s | r | diag");
  }
}
