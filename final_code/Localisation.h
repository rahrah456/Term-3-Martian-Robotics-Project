#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Map.h"

// ============================================================
//  LOCALISATION  —  dead-reckoning with opportunistic correction
//  Pose tracked relative to base origin (mm, degrees).
//
//  Heading:  pure gyro dead-reckoning (no magnetometer).
//
//  Position:  encoder primary, accelerometer blended only when
//    encoder-vs-accel slip discrepancy exceeds threshold.
// ============================================================

struct Pose {
  float x, y;           // mm from base origin
  float headingDeg;     // 0 = +Y, 90 = +X, clockwise
};

class Localisation {
public:
  Pose pose;
  bool valid;

  Localisation() : valid(false) {
    pose.x = 0; pose.y = 0; pose.headingDeg = 0;
  }

  void setPose(float x, float y, float headingDeg) {
    pose.x = x;
    pose.y = y;
    pose.headingDeg = headingDeg;
    valid = true;
  }

  // Reset heading to given value (gyro-only, no mag)
  void resetHeading(float headingDeg) {
    pose.headingDeg = headingDeg;
  }

  // ── Dead-reckoning update ─────────────────────────────────
  // Call every 20ms with encoder deltas, gyro yaw rate,
  // and body-frame accelerometer (g).
  void update(long encL, long encR,
              float gyroZ,
              float accX, float accY,
              float pitch, float roll) {
    if (!valid) return;

    static long lastEncL = encL, lastEncR = encR;
    long dL = encL - lastEncL;
    long dR = encR - lastEncR;
    lastEncL = encL;
    lastEncR = encR;

    unsigned long now = micros();
    static unsigned long lastUs = 0;
    float dt = (now - lastUs) / 1000000.0f;
    lastUs = now;
    if (dt <= 0 || dt > 0.05) dt = 0.02;

    // ── Heading: pure gyro integration ──────────────────────
    pose.headingDeg += gyroZ * dt;
    if (pose.headingDeg < 0.0f) pose.headingDeg += 360.0f;
    if (pose.headingDeg >= 360.0f) pose.headingDeg -= 360.0f;

    float rad = pose.headingDeg * DEG_TO_RAD;

    // ── Position: encoder primary ───────────────────────────
    float avgTicks = (dL + dR) / 2.0f;
    float fwdDistMM = ticksToMm((long)(avgTicks >= 0 ? avgTicks : -avgTicks));
    if (avgTicks < 0) fwdDistMM = -fwdDistMM;
    float encVel = fwdDistMM / dt;   // mm/s

    pose.x += fwdDistMM * sinf(rad);
    pose.y += fwdDistMM * cosf(rad);

    // ── Slip detection + adaptive accel blend ───────────────
    static float prevEncVel = 0;
    float encAccel = (encVel - prevEncVel) / dt;  // mm/s²
    prevEncVel = encVel;

    // ── Forward acceleration from IMU (body → world frame, gravity removed) ──
    // Gravity component in body frame: g_X = -sin(pitch), g_Y = cos(pitch)*sin(roll)
    float linX = accX + sinf(pitch);
    float linY = accY - cosf(pitch) * sinf(roll);
    static EMA accelSmooth(0.05f);     // ~0.4s EMA at 50Hz
    float rawFwdAcc = (linX * cosf(rad) + linY * sinf(rad)) * 9810.0f;  // g → mm/s²
    float imuFwdAcc = accelSmooth.update(rawFwdAcc);

    // Slip metric: |encoder accel - IMU accel| normalised
    float slip = fabsf(encAccel - imuFwdAcc);
    float alpha = 0.0f;   // accel blend fraction (0 = none)

    if (slip > 300.0f) {  // ~0.3 m/s² discrepancy → possible slip
      // Blend accelerometer in proportion to slip magnitude
      alpha = constrain((slip - 300.0f) / 3000.0f, 0.02f, 0.15f);

      // Double-integrate IMU acceleration for slip correction
      static float aidVel = 0, aidDisp = 0;
      aidVel += imuFwdAcc * dt;
      aidVel *= 0.90f;   // damp drift
      aidDisp += aidVel * dt;

      pose.x += alpha * aidDisp * sinf(rad);
      pose.y += alpha * aidDisp * cosf(rad);
      aidDisp -= alpha * aidDisp;   // remove blended portion
    }
  }

  // ── Correction: RFID tag hit ──────────────────────────────
  void correctFromRFID(const ArenaMap& map, uint8_t row, uint8_t col) {
    PointI pt = map.holeCentre(row, col);
    pose.x = pt.x;
    pose.y = pt.y;
  }

  // ── Distance to target ────────────────────────────────────
  float distanceTo(float tx, float ty) const {
    float dx = tx - pose.x;
    float dy = ty - pose.y;
    return sqrtf(dx * dx + dy * dy);
  }

  // ── Bearing to target (degrees, 0 = ahead) ────────────────
  float bearingTo(float tx, float ty) const {
    float dx = tx - pose.x;
    float dy = ty - pose.y;
    float targetHeading = atan2f(dx, dy) * RAD_TO_DEG;
    if (targetHeading < 0) targetHeading += 360.0f;
    float diff = targetHeading - pose.headingDeg;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
  }

private:
  // Simple EMA filter (matches the one in Sensors.h)
  struct EMA {
    float val, alpha;
    EMA(float a) : val(0), alpha(a) {}
    float update(float raw) { return val = val * (1.0f - alpha) + raw * alpha; }
  };
};

#endif
