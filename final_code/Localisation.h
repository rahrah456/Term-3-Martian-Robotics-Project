#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Map.h"

// ============================================================
//  LOCALISATION  —  dead-reckoning with opportunistic correction
//  Pose is tracked relative to base origin (mm, degrees).
//  Accelerometer data (g) is blended at 2% to help during
//  encoder slip without introducing long-term drift.
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

  // ── Set known start position (e.g. from base RFID) ────────
  void setPose(float x, float y, float headingDeg) {
    pose.x = x;
    pose.y = y;
    pose.headingDeg = headingDeg;
    valid = true;
  }

  // ── Dead-reckoning update ─────────────────────────────────
  // Call every 20ms with encoder deltas, IMU heading,
  // and body-frame accelerometer values (g).
  void update(long encL, long encR, float imuHeading,
              float accX = 0, float accY = 0) {
    if (!valid) return;

    static long lastEncL = encL, lastEncR = encR;
    long dL = encL - lastEncL;
    long dR = encR - lastEncR;
    lastEncL = encL;
    lastEncR = encR;

    float rad = pose.headingDeg * DEG_TO_RAD;
    pose.headingDeg = imuHeading;

    // ── Primary: encoder odometry ───────────────────────────
    float distMM = ((float)(dL + dR) / 2.0f) / TICKS_PER_M * 1000.0f;
    float encX = distMM * sinf(rad);
    float encY = distMM * cosf(rad);

    // ── Secondary: accelerometer (2% blend) ─────────────────
    // Project body-frame acceleration onto world frame
    float fwdAcc = (accX * cosf(rad) + accY * sinf(rad)) * 9810.0f;  // g → mm/s²

    static float velX = 0, velY = 0;
    static unsigned long lastUs = 0;
    unsigned long nowUs = micros();
    float dt = (nowUs - lastUs) / 1000000.0f;
    lastUs = nowUs;
    if (dt <= 0 || dt > 0.05) dt = 0.02;

    // Integrate acceleration → velocity (damped to prevent drift)
    velX += fwdAcc * sinf(rad) * dt;
    velY += fwdAcc * cosf(rad) * dt;
    velX *= 0.98;   // leaky integrator — damps out bias/drift
    velY *= 0.98;

    // Velocity → displacement (one more integration)
    static float accelDx = 0, accelDy = 0;
    accelDx += velX * dt;
    accelDy += velY * dt;

    // ── Blend: 98% encoder, 2% accelerometer ────────────────
    const float ALPHA = 0.02;
    pose.x += encX + ALPHA * accelDx;
    pose.y += encY + ALPHA * accelDy;
    // Remove blended portion so accelDx doesn't accumulate forever
    accelDx -= ALPHA * accelDx;
    accelDy -= ALPHA * accelDy;

    // ── Slip detection (log if encoder says moving but accel disagrees) ──
    // TODO: use this to adjust blending or trigger correction
    // float accelMag = sqrtf(accX*accX + accY*accY);
    // bool slipping = (fabs(distMM) > 1 && fabs(fwdAcc) < 200);
  }

  // ── Correction: RFID tag hit ──────────────────────────────
  void correctFromRFID(const ArenaMap& map, uint8_t row, uint8_t col) {
    PointI pt = map.holeCentre(row, col);
    pose.x = pt.x;
    pose.y = pt.y;
  }

  // ── Correction: IR line crossing ──────────────────────────
  // TODO: implement once arena line pattern is confirmed.

  // ── Correction: UDS wall distance ─────────────────────────
  // TODO: implement once walls are measured.

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
};

#endif
