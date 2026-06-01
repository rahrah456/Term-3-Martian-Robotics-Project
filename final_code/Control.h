#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <Motoron.h>
#include <Servo.h>
#include "Config.h"
#include "Sensors.h"

// ============================================================
//  CONTROL  —  motor driver, PID, non-blocking MotionSM
// ============================================================

// Encoder globals — defined in final_code.ino
extern volatile long encL;
extern volatile long encR;
extern bool motorsRunning;           // tracked for heading filter

// Forward declaration (defined later in this file)
bool obstacleAhead(float udsMidCm);

// ── Motor Helper ────────────────────────────────────────────
void setMotors(MotoronI2C& mc, int left, int right) {
  // Bias: right track gets more power to compensate for left being stronger
  right = (int)(right * BIAS_RIGHT);
  left  = constrain(left,  -MOTOR_MAX, MOTOR_MAX);
  right = constrain(right, -MOTOR_MAX, MOTOR_MAX);
  mc.setSpeed(2, -left);    // M2 = left track (polarity reversed)
  mc.setSpeed(1, right);    // M1 = right track
  motorsRunning = (left != 0 || right != 0);
}

// ── Velocity PID Controller ─────────────────────────────────
struct PIDSpeed {
  float integral, prevErr, kp, ki, kd;
  unsigned long lastMicros;
  long lastEnc;

  PIDSpeed() : integral(0), prevErr(0), kp(0.8), ki(0.15), kd(0.05),
               lastMicros(0), lastEnc(0) {}
  PIDSpeed(float p, float i, float d)
    : integral(0), prevErr(0), kp(p), ki(i), kd(d),
      lastMicros(0), lastEnc(0) {}

  void reset(long encNow) {
    integral = 0; prevErr = 0; lastEnc = encNow; lastMicros = micros();
  }

  int update(long encNow, int target) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.02f;
    float actual = (encNow - lastEnc) / dt;
    lastEnc = encNow;
    lastMicros = now;
    float err = (float)target - actual;
    integral += err * dt;
    integral = constrain(integral, -400.0f, 400.0f);
    float deriv = (err - prevErr) / dt;
    prevErr = err;
    return constrain((int)(kp * err + ki * integral + kd * deriv), -800, 800);
  }
};

// ── Non-Blocking Motion State Machine ───────────────────────
// The main loop ticks this once per iteration.  Sensor data is
// passed in from outside so the loop always stays responsive.
//
// Usage:
//   MotionSM m;
//   m.startStraight(speed, ticks);
//   while (m.tick(…) == RUNNING) { … read sensors … delay(20); }

struct MotionSM {
  enum Result { DONE = 0, RUNNING = -1, TIMEOUT = 1, BLOCKED = 2 };
  enum Type { IDLE, STRAIGHT, TURN, LINE_FOLLOW, WALL_FOLLOW, CENTRE_TUNNEL, AVOID_OBSTACLE };

  Type type = IDLE;
  int result = DONE;

  // ── Common state ──────────────────────────────────────────
  PIDSpeed pidL, pidR;
  long startL = 0, startR = 0, ticks = 0;
  int speed = 0, dir = 0;
  unsigned long deadline = 0;

  // ── Line / wall follow state ──────────────────────────────
  float integral = 0, prevErr = 0;
  unsigned long prevTime = 0;
  unsigned long startMs = 0, stopAfterMs = 0;
  int baseSpeed = 0;
  int maxDiff = 120;
  float kp = 30, ki = 1.0, kd = 0.0;

  // Wall follow specific
  int wallSide = 0;
  float cosAngle = 0.5299f;
  float wallTargetDist = 20.0f;
  float buf[5] = {0};
  int bufIdx = 0;
  bool bufFull = false;

  // Line follow hole handling
  bool wasHole = false;
  unsigned long holeStart = 0;
  int holdL = 0, holdR = 0;

  // Avoidance sequence sub-states
  int avoidPhase; // 0 = pick direction + spin, 1 = hug side, 2 = clear + swing, 3 = line center
  int avoidDirection = 0; // 1 = CW (hug left), -1 = CCW (hug right)
  int avoidClearCount;    // consecutive >35cm readings for phase 1 exit debounce
  long avoidStartEnc;     // encoder average at avoidance start, for min spin distance
  long avoidPhase2Enc;    // encoder snapshot for phase 2 sub-phases
  bool avoidPhase2Swing;  // true = currently in the swing portion of phase 2
  unsigned long phaseStartMs;

  // Turn accel tracking: set to 180000 during turns, restore to 800 after
  bool turnAccelSet = false;

  void setAccel(MotoronI2C& mc, int val) {
    mc.setMaxAcceleration(1, val);
    mc.setMaxDeceleration(1, val);
    mc.setMaxAcceleration(2, val);
    mc.setMaxDeceleration(2, val);
  }

  // ── Start helpers ─────────────────────────────────────────

  void startStraight(int spd, long tgt) {
    type = STRAIGHT; speed = spd; ticks = tgt;
    deadline = millis() + 15000;
    pidL.reset(encL); pidR.reset(encR);
    startL = encL; startR = encR;
    result = RUNNING;
    turnAccelSet = false;
  }

  void startTurn(int d, int spd, long tgt) {
    type = TURN; dir = d; speed = spd; ticks = tgt;
    deadline = millis() + 15000;
    pidL.reset(encL); pidR.reset(encR);
    startL = encL; startR = encR;
    result = RUNNING;
    turnAccelSet = false;
  }

  void startLineFollow(int base, float p, float i, float d, int md,
                       unsigned long stopMs = 15000) {
    type = LINE_FOLLOW; baseSpeed = base; stopAfterMs = stopMs;
    kp = p; ki = i; kd = d; maxDiff = md;
    integral = 0; prevErr = 0; prevTime = millis();
    startMs = millis();
    wasHole = false; holdL = base; holdR = base;
    result = RUNNING;
  }

  void startWallFollow(int base, int side, float targetCm, float p, float i, float d, int md,
                       unsigned long stopMs = 15000) {
    type = WALL_FOLLOW; baseSpeed = base; stopAfterMs = stopMs;
    wallSide = side; wallTargetDist = targetCm;
    kp = p; ki = i; kd = d; maxDiff = md;
    cosAngle = 0.5299f;
    integral = 0; prevErr = 0; prevTime = millis(); startMs = millis();
    bufIdx = 0; bufFull = false;
    result = RUNNING;
  }

  void startTunnelCentre(int base, float p, int md,
                         unsigned long stopMs = 15000) {
    type = CENTRE_TUNNEL; baseSpeed = base; stopAfterMs = stopMs;
    kp = p; maxDiff = md;
    startMs = millis();
    result = RUNNING;
  }

  void startAvoid(int speed) {
    type = AVOID_OBSTACLE;
    result = RUNNING;
    startMs = millis();
    phaseStartMs = millis();
    baseSpeed = speed;
    avoidPhase = 0;
    avoidDirection = 0;  // 0 = unset, phase 0 chooses based on side UDS
    avoidClearCount = 0;
    avoidStartEnc = (abs(encL) + abs(encR)) / 2;
    avoidPhase2Enc = 0;
    avoidPhase2Swing = false;
  }
  void stop() {
    type = IDLE; result = DONE;
  }

  // ── Tick: advance one step ───────────────────────────────
  // Pass sensor data needed by the active motion type.
  //   STRAIGHT/TURN:  use encL/encR globals directly; obstacle check uses udsDistCm
  //   LINE_FOLLOW:    pass centroid
  //   WALL_FOLLOW:    uses udsLeftCm (side<0) or udsRightCm (side>0)
  //   CENTRE_TUNNEL:  uses udsLeftCm, udsRightCm
  //   AVOID_OBSTACLE: uses udsLeftCm, udsRightCm, udsDistCm; phase 3 uses centroid
  //
  // Returns DONE, RUNNING, TIMEOUT, or BLOCKED.

  int tick(MotoronI2C& mc, int centroid = -1,
           float udsDistCm = -1, float udsLeftCm = -1, float udsRightCm = -1) {
    if (type == IDLE) {
      if (turnAccelSet) { setAccel(mc, MOTOR_RAMP); turnAccelSet = false; }
      return DONE;
    }
    if (result != RUNNING) return result;

    // Obstacle check: only for open-loop STRAIGHT/TURN (not controlled types
    // like WALL_FOLLOW which intentionally get close to walls).
    if (type == STRAIGHT || type == TURN) {
      if (obstacleAhead(udsDistCm)) {
        setMotors(mc, 0, 0);
        if (type == TURN && turnAccelSet) { setAccel(mc, MOTOR_RAMP); turnAccelSet = false; }
        return result = BLOCKED;
      }
    }

    switch (type) {

      // ── STRAIGHT (open-loop, no PID) ───────────────────────
      case STRAIGHT: {
        if (turnAccelSet) { setAccel(mc, MOTOR_RAMP); turnAccelSet = false; }
        if (millis() >= deadline) { setMotors(mc, 0, 0); return result = TIMEOUT; }
        if ((abs(encL - startL) + abs(encR - startR)) / 2 >= ticks)
          { setMotors(mc, 0, 0); return result = DONE; }
        setMotors(mc, speed, speed);
        return RUNNING;
      }

      // ── TURN (open-loop, no PID) ──────────────────────────
      case TURN: {
        if (!turnAccelSet) { setAccel(mc, 180000); turnAccelSet = true; }
        if (millis() >= deadline) {
          setMotors(mc, 0, 0);
          if (turnAccelSet) { setAccel(mc, MOTOR_RAMP); turnAccelSet = false; }
          return result = TIMEOUT;
        }
        if ((abs(encL - startL) + abs(encR - startR)) / 2 >= ticks) {
          setMotors(mc, 0, 0);
          if (turnAccelSet) { setAccel(mc, MOTOR_RAMP); turnAccelSet = false; }
          return result = DONE;
        }
        setMotors(mc, dir * speed, -dir * speed);
        return RUNNING;
      }

      // ── LINE FOLLOW ───────────────────────────────────────
      case LINE_FOLLOW: {
        if (millis() - startMs >= stopAfterMs)
          { setMotors(mc, 0, 0); return result = DONE; }

        if (centroid < 0) {
          if (!wasHole) {
            wasHole = true; holeStart = millis();
            setMotors(mc, holdL, holdR);
            return RUNNING;
          }
          if (millis() - holeStart < 200) {
            setMotors(mc, holdL, holdR);
            return RUNNING;
          }
          setMotors(mc, 0, 0);
          return result = DONE;   // lost line
        }
        wasHole = false;

        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

        float error = (float)centroid - 4000.0f;
        integral += error * dt;
        integral = constrain(integral, -50.0f, 50.0f);
        float deriv = (error - prevErr) / dt;
        prevErr = error;

        float correction = kp * error + ki * integral + kd * deriv;
        correction = constrain(correction, -(float)maxDiff, (float)maxDiff);
        int left  = constrain(baseSpeed + (int)correction, MOTOR_MIN, MOTOR_MAX);
        int right = constrain(baseSpeed - (int)correction, MOTOR_MIN, MOTOR_MAX);
        holdL = left; holdR = right;
        setMotors(mc, left, right);
        return RUNNING;
      }

      // ── WALL FOLLOW ───────────────────────────────────────
      // Uses the side UDS: udsLeftCm for wallSide<0, udsRightCm for wallSide>0
      case WALL_FOLLOW: {
        if (millis() - startMs >= stopAfterMs)
          { setMotors(mc, 0, 0); return result = DONE; }

        float sideDist = (wallSide < 0) ? udsLeftCm : udsRightCm;
        buf[bufIdx] = sideDist;
        bufIdx = (bufIdx + 1) % 5;
        if (bufIdx == 0) bufFull = true;
        float dist = bufFull ? medianOf5(buf[0], buf[1], buf[2], buf[3], buf[4])
                             : buf[max(0, bufIdx - 1)];

        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

        float error = dist * cosAngle - wallTargetDist;
        integral += error * dt;
        integral = constrain(integral, -150.0f, 150.0f);
        float deriv = (error - prevErr) / dt;
        prevErr = error;

        float correction = kp * error + ki * integral + kd * deriv;
        correction = constrain(correction, -(float)maxDiff, (float)maxDiff);
        int left  = constrain(baseSpeed + wallSide * (int)correction, MOTOR_MIN, MOTOR_MAX);
        int right = constrain(baseSpeed - wallSide * (int)correction, MOTOR_MIN, MOTOR_MAX);
        setMotors(mc, left, right);
        return RUNNING;
      }

      // ── CENTRE TUNNEL ─────────────────────────────────────
      case CENTRE_TUNNEL: {
        if (millis() - startMs >= stopAfterMs)
          { setMotors(mc, 0, 0); return result = DONE; }

        float error = udsLeftCm - udsRightCm;
        float correction = kp * error;
        correction = constrain(correction, -(float)maxDiff, (float)maxDiff);
        int left  = constrain(baseSpeed + (int)correction, MOTOR_MIN, MOTOR_MAX);
        int right = constrain(baseSpeed - (int)correction, MOTOR_MIN, MOTOR_MAX);
        setMotors(mc, left, right);
        return RUNNING;
      }

      // ── SENSOR-DRIVEN OBSTACLE DETOUR SEQUENCE ─────────────────
      case AVOID_OBSTACLE: {
        switch (avoidPhase) {
          case 0: {
            // Phase 0: pick direction based on side clearance, then spin until clear
            if (avoidDirection == 0) {
              float diff = udsLeftCm - udsRightCm;
              avoidDirection = (diff > 15.0f) ? 1 : -1;
              avoidStartEnc = (abs(encL) + abs(encR)) / 2;
            }
            long avgEnc = (abs(encL) + abs(encR)) / 2;
            bool minSpinOk = (avgEnc - avoidStartEnc) >= 900;  // ~150°
            if ((minSpinOk && udsDistCm >= 25.0f) || millis() - phaseStartMs >= 2000) {
              avoidPhase = 1;
              phaseStartMs = millis();
            } else {
              setMotors(mc, avoidDirection * baseSpeed,
                            -avoidDirection * baseSpeed);
            }
            break;
          }

          case 1: {
            // Phase 1: Hug the correct side (avoidDirection > 0 → left, else right)
            float sideDist = (avoidDirection > 0) ? udsLeftCm : udsRightCm;
            if (sideDist > 35.0f) {
              avoidClearCount++;
              if (avoidClearCount >= 3) {
                avoidPhase = 2;
                phaseStartMs = millis();
                break;
              }
            } else {
              avoidClearCount = 0;
            }

            // Proportional control to stay exactly 15.0 cm from obstacle
            float error = sideDist - 15.0f;
            float kpSide = 12.0f;
            int correction = (int)(error * kpSide);
            int left  = constrain(baseSpeed + avoidDirection * correction,
                                  MOTOR_MIN, MOTOR_MAX);
            int right = constrain(baseSpeed - avoidDirection * correction,
                                  MOTOR_MIN, MOTOR_MAX);
            setMotors(mc, left, right);
            break;
          }

          case 2: {
            // Phase 2: drive past, then swing back opposite to initial turn
            long avgEnc = (abs(encL) + abs(encR)) / 2;

            if (!avoidPhase2Swing) {
              // Forward: 80 mm to clear the rear corner
              if (avoidPhase2Enc == 0) avoidPhase2Enc = avgEnc;
              long fwdTicks = ticksForDistance(80);
              if (avgEnc - avoidPhase2Enc >= fwdTicks) {
                avoidPhase2Swing = true;
                avoidPhase2Enc = avgEnc;
              } else {
                setMotors(mc, baseSpeed, baseSpeed);
              }
            }

            if (avoidPhase2Swing) {
              // Swing ~80° back
              long swingTicks = ticksForTurn(80);
              if (avgEnc - avoidPhase2Enc >= swingTicks) {
                avoidPhase2Enc = 0;
                avoidPhase2Swing = false;
                avoidPhase = 3;
                phaseStartMs = millis();
              } else {
                setMotors(mc, -avoidDirection * baseSpeed,
                              avoidDirection * baseSpeed);
              }
            }
            break;
          }

          case 3: {
            // Phase 3: find and center on a grid line
            unsigned long duration = millis() - phaseStartMs;
            if (centroid < 0) {
              if (duration < 5000) {
                setMotors(mc, MOTOR_MIN + 60, MOTOR_MIN + 60);
              } else {
                setMotors(mc, 0, 0);
                return result = DONE;
              }
            } else {
              float error = (float)centroid - 4000.0f;
              if (fabsf(error) < 200.0f) {
                setMotors(mc, 0, 0);
                return result = DONE;
              }
              int correction = constrain((int)(error * 0.15f), -60, 60);
              setMotors(mc, MOTOR_MIN + 60 + correction, MOTOR_MIN + 60 - correction);
            }
            break;
          }
        }
        return RUNNING;
      }

      default:
        return result = DONE;
    }
  }
};

// ── Seed Tracking ───────────────────────────────────────────
extern int g_seedIdx;   // current servo index (0=locked, 1-5 dispensed next)

// ── Seed Dispensing ─────────────────────────────────────────
// All functions are non-blocking — servo.write() sets the target
// and returns immediately; the servo moves asynchronously.

void lockSeeds(Servo& servo) {
  g_seedIdx = 0;
  servo.write(SEED_ANGLES[0]);
}

void dispenseSeed(Servo& servo, int position) {
  if (position < 0 || position >= SEED_COUNT) return;
  g_seedIdx = position;
  servo.write(SEED_ANGLES[position]);
}

void dispenseNextSeed(Servo& servo) {
  if (g_seedIdx < 1 || g_seedIdx >= SEED_COUNT) return;
  servo.write(SEED_ANGLES[g_seedIdx]);
  g_seedIdx++;
  if (g_seedIdx >= SEED_COUNT) g_seedIdx = SEED_COUNT - 1;
}

// ── Obstacle Detection ──────────────────────────────────────
bool obstacleAhead(float udsMidCm) {
  return udsMidCm > 0 && udsMidCm < (OBSTACLE_STOP_MM / 10);
}

// ── Revive Move ─────────────────────────────────────────────
// Open-loop straight with linear deceleration from startSpeed to
// endSpeed.  Motors may stall on contact — overshoot is fine.
struct ReviveMove {
  long startEnc = 0, targetTicks = 0;
  int startSpeed = 500, endSpeed = 350;
  bool running = false;

  void start(long target) {
    running = true;
    targetTicks = target;
    startEnc = (abs(encL) + abs(encR)) / 2;
  }

  int tick(MotoronI2C& mc) {
    if (!running) return MotionSM::DONE;
    long avgEnc = (abs(encL) + abs(encR)) / 2;
    long done = avgEnc - startEnc;
    if (done >= targetTicks) {
      setMotors(mc, 0, 0);
      running = false;
      return MotionSM::DONE;
    }
    float frac = (float)done / targetTicks;
    int spd = startSpeed - (int)((startSpeed - endSpeed) * frac);
    spd = constrain(spd, endSpeed, startSpeed);
    setMotors(mc, spd, spd);
    return MotionSM::RUNNING;
  }

  void stop() {
    running = false;
  }
};

#endif
