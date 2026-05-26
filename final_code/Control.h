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

// Forward declaration (defined later in this file)
bool obstacleAhead();

// ── Motor Helper ────────────────────────────────────────────
void setMotors(MotoronI2C& mc, int left, int right) {
  left  = constrain(left,  -MOTOR_MAX, MOTOR_MAX);
  right = constrain(right, -MOTOR_MAX, MOTOR_MAX);
  mc.setSpeed(1, left);
  mc.setSpeed(2, -right);
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
  enum Type { IDLE, STRAIGHT, TURN, LINE_FOLLOW, WALL_FOLLOW, CENTRE_TUNNEL };

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
  int maxDiff = 40;
  float kp = 0.5, ki = 0.0, kd = 0.0;

  // Wall follow specific
  int wallSide = 0;
  int trigPin = 0, echoPin = 0;
  float cosAngle = 0.5299f;
  float buf[5] = {0};
  int bufIdx = 0;
  bool bufFull = false;

  // Line follow hole handling
  bool wasHole = false;
  unsigned long holeStart = 0;
  int holdL = 0, holdR = 0;

  // ── Start helpers ─────────────────────────────────────────

  void startStraight(int spd, long tgt) {
    type = STRAIGHT; speed = spd; ticks = tgt;
    deadline = millis() + 15000;
    pidL.reset(encL); pidR.reset(encR);
    startL = encL; startR = encR;
    result = RUNNING;
  }

  void startTurn(int d, int spd, long tgt) {
    type = TURN; dir = d; speed = spd; ticks = tgt;
    deadline = millis() + 15000;
    pidL.reset(encL); pidR.reset(encR);
    startL = encL; startR = encR;
    result = RUNNING;
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

  void startWallFollow(int base, int side, float p, float i, float d, int md,
                       unsigned long stopMs = 15000) {
    type = WALL_FOLLOW; baseSpeed = base; stopAfterMs = stopMs;
    wallSide = side;
    kp = p; ki = i; kd = d; maxDiff = md;
    trigPin = (side < 0) ? PIN_UDS_LT : PIN_UDS_RT;
    echoPin = (side < 0) ? PIN_UDS_LE : PIN_UDS_RE;
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

  void stop() {
    type = IDLE; result = DONE;
  }

  // ── Tick: advance one step ───────────────────────────────
  // Pass sensor data needed by the active motion type.
  //   STRAIGHT/TURN: use encL/encR globals directly
  //   LINE_FOLLOW:   pass centroid
  //   WALL_FOLLOW:   pass udsDistCm
  //   CENTRE_TUNNEL: pass udsLeftCm, udsRightCm
  //
  // Returns DONE, RUNNING, TIMEOUT, or BLOCKED.

  int tick(MotoronI2C& mc, int centroid = -1,
           float udsDistCm = -1, float udsLeftCm = -1, float udsRightCm = -1) {
    if (type == IDLE) return DONE;
    if (result != RUNNING) return result;

    if (obstacleAhead()) { setMotors(mc, 0, 0); return result = BLOCKED; }

    switch (type) {

      // ── STRAIGHT ──────────────────────────────────────────
      case STRAIGHT: {
        if (millis() >= deadline) { setMotors(mc, 0, 0); return result = TIMEOUT; }
        if (abs(encL - startL) >= ticks && abs(encR - startR) >= ticks)
          { setMotors(mc, 0, 0); return result = DONE; }
        int cmdL = pidL.update(encL, speed);
        int cmdR = pidR.update(encR, speed);
        setMotors(mc, cmdL, cmdR);
        return RUNNING;
      }

      // ── TURN ──────────────────────────────────────────────
      case TURN: {
        if (millis() >= deadline) { setMotors(mc, 0, 0); return result = TIMEOUT; }
        if (abs(encL - startL) >= ticks && abs(encR - startR) >= ticks)
          { setMotors(mc, 0, 0); return result = DONE; }
        int cmdL = pidL.update(encL,  dir * speed);
        int cmdR = pidR.update(encR, -dir * speed);
        setMotors(mc, cmdL, cmdR);
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
      case WALL_FOLLOW: {
        if (millis() - startMs >= stopAfterMs)
          { setMotors(mc, 0, 0); return result = DONE; }

        if (udsDistCm < 0) udsDistCm = (float)readUDS(trigPin, echoPin);

        buf[bufIdx] = udsDistCm;
        bufIdx = (bufIdx + 1) % 5;
        if (bufIdx == 0) bufFull = true;
        float dist = bufFull ? medianOf5(buf[0], buf[1], buf[2], buf[3], buf[4])
                             : buf[max(0, bufIdx - 1)];

        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

        float error = dist * cosAngle - 20.0f;
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

        if (udsLeftCm < 0)  udsLeftCm  = (float)readUDS(PIN_UDS_LT, PIN_UDS_LE);
        if (udsRightCm < 0) udsRightCm = (float)readUDS(PIN_UDS_RT, PIN_UDS_RE);

        float error = udsLeftCm - udsRightCm;
        float correction = kp * error;
        correction = constrain(correction, -(float)maxDiff, (float)maxDiff);
        int left  = constrain(baseSpeed + (int)correction, MOTOR_MIN, MOTOR_MAX);
        int right = constrain(baseSpeed - (int)correction, MOTOR_MIN, MOTOR_MAX);
        setMotors(mc, left, right);
        return RUNNING;
      }

      default:
        return result = DONE;
    }
  }
};

// ── Seed Dispensing ─────────────────────────────────────────
void dispenseSeed(Servo& servo, int position) {
  if (position < 0 || position >= SEED_COUNT) return;
  servo.write(SEED_ANGLES[position]);
  delay(500);
}

void lockSeeds(Servo& servo) {
  servo.write(SEED_ANGLES[0]);
  delay(300);
}

// ── Obstacle Detection ──────────────────────────────────────
bool obstacleAhead() {
  long d = readUDS(PIN_UDS_MT, PIN_UDS_ME);
  return d > 0 && d < (OBSTACLE_STOP_MM / 10);
}

#endif
