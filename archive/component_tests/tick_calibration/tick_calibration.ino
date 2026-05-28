// ============================================================
//  TICK CALIBRATION  —  forward / turn by distance or angle
//  Open-loop (demo 9 pattern).  2kHz encoder polling.
//
//  Commands (send via Serial Monitor, 115200 baud):
//    d <ticks> [trials] — forward by <ticks>, repeated <trials> times (MOVE_SPEED=500)
//    a <ticks> [trials] — tank turn by <ticks>, repeated <trials> times (TURN_SPEED=660)
//    f <ticks> <speed>  — forward by raw ticks (custom speed)
//    t <ticks> <speed>  — tank turn by raw ticks (custom speed)
//    s                  — stop
//    r                  — reset encoders
//    diag               — encoder diagnostics (5s)
//
//  d / a with trials > 1 runs the move multiple times with 400ms
//  pause between, then prints the average left/right ticks across
//  all trials.  Measure the actual distance/angle with a ruler and
//  record (commanded_ticks, mm_travelled) to fit DIST_TICK_M/_C
//  and TURN_TICK_M/_C in Config.h.
// ============================================================
// Instructions
// 1. measure motor bias
//   1.1 make it go 2 metres (should be ~2000 ticks) in a straight line
//   1.2 use the ratio of tick outputs to update BIAS_RIGHT. BIAS_RIGHT = left ticks / right ticks
//   1.3 repeat striahgt line to confirm straight

// 2. ticks per metre calibration
//   2.1 make it go 2 metres (should be ~2000 ticks) in a straight line
//   2.2 measure distance
//   2.3 use this to update TICKS_PER_M

// 3. gather many data points
//   3.1 for both turning (a) and moving (d)
//   3.2 reset tick counters using r command
//   3.3 give different move commands and measure the input and output in a table (e.g. d 200 then record 200 and the outputted ticks)
//   3.4 focus your data points around small moves but include larger moves as well
//   3.5 preferably test on arena, but floor is fine



// ============================================================


#include <Arduino.h>
#include <Motoron.h>

// ── Pins ────────────────────────────────────────────────────
const int ENC_LA = 4, ENC_LB = 5;    // left track
const int ENC_RA = 2, ENC_RB = 3;    // right track
const int KILL_BTN = 38;
const int ACT_LED = 39;

// ── Calibration (mirrors Config.h — update after regression) ─
// const float TICKS_PER_M = 3647.0; // 3607
// const long  TICKS_PER_90 = 525;    
const int   MOVE_SPEED = 500;
const int   TURN_SPEED = 660;
const float BIAS_RIGHT = 1.047f; // 

// (convenience wrappers no longer needed — use raw ticks directly)

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
// Returns actual absolute ticks for left and right via references.
void runOL(int left, int right, long targetTicks, long& actualL, long& actualR) {
  encL = 0; encR = 0;
  setMotors(left, right);

  unsigned long lastPoll = 0;

  while (true) {
    unsigned long now = micros();
    if (now - lastPoll >= 500) { lastPoll = now; pollEncoders(); }
    handleEStop();
    if (killed) { setMotors(0, 0); actualL = abs(encL); actualR = abs(encR); return; }
    if ((abs(encL) + abs(encR)) / 2 >= targetTicks) break;
  }
  setMotors(0, 0);
  actualL = abs(encL);
  actualR = abs(encR);
}

// ── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Tick Calibration (open-loop, 2kHz polling) ===");
  Serial.println("  d <ticks> [trials]  forward by <ticks>, <trials> times");
  Serial.println("  a <ticks> [trials]  tank turn by <ticks>, <trials> times");
  Serial.println("  f <ticks> <speed>   forward raw ticks (custom speed)");
  Serial.println("  t <ticks> <speed>   turn raw ticks (custom speed)");
  Serial.println("  s                   stop");
  Serial.println("  r                   reset encoders");
  Serial.println("  diag                encoder diagnostics");
  Serial.println();
  Serial.println("d/a with trials>1 repeats with 400ms pause and averages L/R ticks.");

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

  else if (cmd == 'd') {
    long ticks; int trials;
    char buf[32]; arg.toCharArray(buf, sizeof(buf));
    int n = sscanf(buf, "%ld %d", &ticks, &trials);
    if (n < 1 || ticks <= 0) { Serial.println("? use: d <ticks> [trials]"); return; }
    if (n < 2) trials = 1;
    else trials = max(1, trials);
    Serial.print("Forward "); Serial.print(ticks); Serial.print(" ticks x "); Serial.print(trials); Serial.println(" trials");

    long sumL = 0, sumR = 0;
    for (int t = 0; t < trials; t++) {
      long aL, aR;
      runOL(MOVE_SPEED, MOVE_SPEED, ticks, aL, aR);
      sumL += aL; sumR += aR;
      Serial.print("  trial "); Serial.print(t + 1);
      Serial.print(": L="); Serial.print(aL); Serial.print(" R="); Serial.println(aR);
      if (t < trials - 1) delay(400);
    }
    float avgL = (float)sumL / trials;
    float avgR = (float)sumR / trials;
    float avg  = (avgL + avgR) / 2.0f;
    Serial.print("RESULT: avgL="); Serial.print(avgL, 1);
    Serial.print("  avgR="); Serial.print(avgR, 1);
    Serial.print("  avg="); Serial.println(avg, 1);
  }
  else if (cmd == 'a') {
    mc.setMaxAcceleration(1, 180000);
    mc.setMaxDeceleration(1, 180000);
    mc.setMaxAcceleration(2, 180000);
    mc.setMaxDeceleration(2, 180000);
    long ticks; int trials;
    char buf[32]; arg.toCharArray(buf, sizeof(buf));
    int n = sscanf(buf, "%ld %d", &ticks, &trials);
    if (n < 1 || ticks <= 0) { Serial.println("? use: a <ticks> [trials]"); return; }
    if (n < 2) trials = 1;
    else trials = max(1, trials);
    Serial.print("Turn "); Serial.print(ticks); Serial.print(" ticks x "); Serial.print(trials); Serial.println(" trials");

    long sumL = 0, sumR = 0;
    for (int t = 0; t < trials; t++) {
      long aL, aR;
      runOL(-TURN_SPEED, TURN_SPEED, ticks, aL, aR);
      sumL += aL; sumR += aR;
      Serial.print("  trial "); Serial.print(t + 1);
      Serial.print(": L="); Serial.print(aL); Serial.print(" R="); Serial.println(aR);
      if (t < trials - 1) delay(400);
    }
    float avgL = (float)sumL / trials;
    float avgR = (float)sumR / trials;
    float avg  = (avgL + avgR) / 2.0f;
    Serial.print("RESULT: avgL="); Serial.print(avgL, 1);
    Serial.print("  avgR="); Serial.print(avgR, 1);
    Serial.print("  avg="); Serial.println(avg, 1);

    mc.setMaxAcceleration(1, 800);
    mc.setMaxDeceleration(1, 800);
    mc.setMaxAcceleration(2, 800);
    mc.setMaxDeceleration(2, 800);
  }

  else if (cmd == 'f') {
    long ticks = (long)val;
    int speed = MOVE_SPEED;
    int secondSpace = arg.indexOf(' ');
    if (secondSpace > 0) { speed = constrain((int)arg.substring(secondSpace + 1).toFloat(), 0, 660); ticks = (long)arg.substring(0, secondSpace).toFloat(); }
    long aL, aR;
    runOL(speed, speed, ticks, aL, aR);
    Serial.print("L:"); Serial.print(aL); Serial.print("  R:"); Serial.print(aR);
    Serial.print("  target="); Serial.println(ticks);
  }
  else if (cmd == 't') {
    long ticks = (long)val;
    int speed = TURN_SPEED;
    int secondSpace = arg.indexOf(' ');
    if (secondSpace > 0) { speed = constrain((int)arg.substring(secondSpace + 1).toFloat(), 0, 660); ticks = (long)arg.substring(0, secondSpace).toFloat(); }
    long aL, aR;
    runOL(-speed, speed, ticks, aL, aR);
    Serial.print("L:"); Serial.print(aL); Serial.print("  R:"); Serial.print(aR);
    Serial.print("  target="); Serial.println(ticks);
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
    Serial.println("?  use: d <ticks> [trials] | a <ticks> [trials] | f <ticks> <speed> | t <ticks> <speed> | s | r | diag");
  }
}
