#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
// #include <MFRC522_I2C.h>
#include "Config.h"

// ── Magnetometer calibration (from archive spherical-fit sketch) ──
// RE-CALIBRATED MICROTESLA PARAMETERS (49.1579 uT FIELD). Copied verbatim
// from archive/component_tests/magnetometer_spherical_fitted_heading/.
static const float hard_iron_bias_x = -92.425254;
static const float hard_iron_bias_y = 74.120214;
static const float hard_iron_bias_z = -1.234978;

static const double soft_iron_bias_xx = 0.928803;
static const double soft_iron_bias_xy = 0.023684;
static const double soft_iron_bias_xz = 0.070527;

static const double soft_iron_bias_yx = 0.023684;
static const double soft_iron_bias_yy = 0.925277;
static const double soft_iron_bias_yz = -0.020476;

static const double soft_iron_bias_zx = 0.070527;
static const double soft_iron_bias_zy = -0.020476;
static const double soft_iron_bias_zz = 0.797892;

// Runtime-adjustable cardinal lookup table (updated by Set N/E/S/W on the
// dashboard).  magLookupRaw[i] = raw mag heading when robot faces grid
// direction i*90°.  Defined in final_code.ino.
extern float magLookupRaw[4];

// Convert a raw tilt-compensated mag heading (clockwise from mag north) into
// a grid heading (0° = grid north = up, clockwise) by lerping through the
// 4-entry cardinal lookup table.
static float rawToGridHeading(float raw) {
  for (int i = 0; i < 4; i++) {
    int j = (i + 1) % 4;
    float start = magLookupRaw[i];
    float end   = magLookupRaw[j];

    // Check if raw falls in segment [start, end) going clockwise
    bool inSeg;
    if (end > start) {
      inSeg = (raw >= start && raw < end);
    } else {
      inSeg = (raw >= start || raw < end);
    }

    if (inSeg) {
      float width  = end - start;
      if (width < 0.0f) width += 360.0f;
      float offset = raw - start;
      if (offset < 0.0f) offset += 360.0f;
      float t = (width > 0.001f) ? (offset / width) : 0.0f;
      float gridH = (float)i * 90.0f + t * 90.0f;
      if (gridH >= 360.0f) gridH -= 360.0f;
      return gridH;
    }
  }
  return raw;  // fallback (shouldn't happen)
}

// ============================================================
//  SENSORS  —  IR, UDS, RFID, IMU, bumper, light sensor
// ============================================================

// ── 1-second EMA helper ──────────────────────────────────────
// alpha ≈ 1 / (freq * tau), at 50Hz and tau=1s → alpha ≈ 0.02
struct EMA {
  float val, alpha;
  EMA(float a = 0.02f) : val(0), alpha(a) {}
  float update(float raw) { return val = val * (1.0f - alpha) + raw * alpha; }
  void reset(float v = 0) { val = v; }
};

// ── IR Array ────────────────────────────────────────────────
// Manual charge-discharge read of 9 pins.

void initIR() {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  pinMode(IR_EMITTER_1, OUTPUT);
  pinMode(IR_EMITTER_2, OUTPUT);
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);
}

static void readRawIR(uint16_t* rawVals) {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
    rawVals[i] = IR_TIMEOUT_US;
  }
  delayMicroseconds(10);
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  unsigned long start = micros();
  uint8_t left = IR_COUNT;
  while (left > 0) {
    unsigned long elapsed = micros() - start;
    if (elapsed >= IR_TIMEOUT_US) break;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      if (rawVals[i] == IR_TIMEOUT_US && digitalRead(IR_PINS[i]) == LOW) {
        rawVals[i] = (uint16_t)elapsed;
        left--;
      }
    }
  }
}

void readIR(uint16_t vals[]) {
  uint16_t rawOff[IR_COUNT];
  uint16_t rawOn[IR_COUNT];

  readRawIR(rawOff);

  digitalWrite(IR_EMITTER_1, HIGH);
  digitalWrite(IR_EMITTER_2, HIGH);
  delayMicroseconds(200);
  readRawIR(rawOn);
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    int32_t adj = (int32_t)rawOn[i] + IR_TIMEOUT_US - (int32_t)rawOff[i];
    if (adj > IR_TIMEOUT_US) adj = IR_TIMEOUT_US;
    if (adj < 0) adj = 0;
    long mapped = map(adj, IR_MINS[i], IR_MAXS[i], 0, 1000);
    vals[i] = constrain(mapped, 0, 1000);
  }
}

int irCentroid(const uint16_t vals[]) {
  uint32_t sum = 0, weighted = 0;
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    uint16_t v = vals[i];
    if (v > 50) {
      sum += v;
      weighted += v * i * 1000;
    }
  }
  if (sum < 100) return -1;
  return (int)(weighted / sum);
}

// ── Ultrasonic Distance Sensors ─────────────────────────────
// Round-robin: reads one sensor per tick via pulseIn with a moderate
// timeout, buffers 3 readings per sensor, outputs median-of-3.
// Blocks for at most UDS_TIMEOUT_US µs per tick (~30ms worst case
// for no echo; usually ~1-5ms for nearby objects).

class UDSManager {
public:
  enum { LEFT = 0, MID = 1, RIGHT = 2, NUM_SENSORS = 3 };

  long distances[NUM_SENSORS];   // median-filtered readings (cm)

  UDSManager() {
    for (int i = 0; i < NUM_SENSORS; i++) {
      distances[i] = UDS_MAX_CM;
      bufPos[i] = 0;
    }
    currentSensor = 0;
  }

  // Call every main loop tick (~20ms).  Reads one UDS via pulseIn,
  // buffers the result, then advances the round-robin.
  void tick() {
    int t = trigPin(currentSensor);
    int e = echoPin(currentSensor);

    digitalWrite(t, LOW);
    delayMicroseconds(2);
    digitalWrite(t, HIGH);
    delayMicroseconds(10);
    digitalWrite(t, LOW);

    long us = pulseIn(e, HIGH, UDS_TIMEOUT_US);
    long val = (us == 0) ? UDS_MAX_CM : us / 58;

    buf[currentSensor][bufPos[currentSensor]] = val;
    bufPos[currentSensor] = (bufPos[currentSensor] + 1) % UDS_MEDIAN_N;

    // Median of last UDS_MEDIAN_N samples
    long s[UDS_MEDIAN_N];
    for (int i = 0; i < UDS_MEDIAN_N; i++)
      s[i] = buf[currentSensor][i];
    for (int i = 0; i < UDS_MEDIAN_N - 1; i++)
      for (int j = 0; j < UDS_MEDIAN_N - 1 - i; j++)
        if (s[j] > s[j + 1]) { long t2 = s[j]; s[j] = s[j + 1]; s[j + 1] = t2; }
    distances[currentSensor] = s[UDS_MEDIAN_N / 2];

    currentSensor = (currentSensor + 1) % NUM_SENSORS;
  }

private:
  int currentSensor;
  static const int UDS_MEDIAN_N = 3;
  long buf[NUM_SENSORS][UDS_MEDIAN_N];
  int bufPos[NUM_SENSORS];

  int trigPin(int idx) {
    return idx == LEFT ? PIN_UDS_LT : idx == MID ? PIN_UDS_MT : PIN_UDS_RT;
  }
  int echoPin(int idx) {
    return idx == LEFT ? PIN_UDS_LE : idx == MID ? PIN_UDS_ME : PIN_UDS_RE;
  }
};

// Legacy single-shot for occasional use (still blocking — use sparingly).
static long readUDS(int trigPin, int echoPin, unsigned long timeoutUs = 4000) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, timeoutUs);
  if (us == 0) return UDS_MAX_CM;
  return us / 58;
}

float medianOf5(float a, float b, float c, float d, float e) {
  float v[5] = {a, b, c, d, e};
  for (int i = 1; i < 5; i++) {
    float key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return v[2];
}

// ── RFID ────────────────────────────────────────────────────
// readRFID() defined in final_code.ino (needs global rfid object).

// ── IMU ─────────────────────────────────────────────────────
// LSM6 accelerometer/gyro + LIS3MDL magnetometer (both on Wire).
// Heading comes from the tilt-compensated magnetometer (accurate to
// ~1°); accel also provides pitch/roll for gravity subtraction.

struct IMUData {
  LSM6    imu;
  LIS3MDL mag;
  float headingDeg;     // grid heading (mag, tilt-compensated, offset-corrected)
  float gyroZ;          // degrees per second (yaw rate)
  float accX, accY, accZ;
  float pitch, roll;    // radians, for gravity subtraction
  bool  ok;
};

void initIMU(IMUData& d) {
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  d.ok = false;
  if (!d.imu.init())   { Serial.println("IMU: lsm6 init failed"); return; }
  d.imu.enableDefault();
  if (!d.mag.init())   { Serial.println("IMU: lis3mdl init failed"); return; }
  d.mag.enableDefault();   // ±4 Gauss default range
  d.ok = true;
  Serial.println("IMU: OK");
}

// ── Tilt-compensated magnetometer heading ───────────────────
// Ported verbatim from the archive spherical-fit sketch (accurate to
// ~1°), except the EMA is applied to the two atan2 arguments instead of
// the wrapped degree output (degree wrapping breaks an EMA on the angle).
// Returns clockwise degrees from magnetic north, with the robot's +X as
// forward. Caller passes to rawToGridHeading() for the grid heading.
static float computeMagHeading(IMUData& d) {
  // 1. Convert raw LSB to microTesla
  float ut_x = (float)d.mag.m.x / 68.42f;
  float ut_y = (float)d.mag.m.y / 68.42f;
  float ut_z = (float)d.mag.m.z / 68.42f;

  // 2. Remove hard-iron offset
  float xm_off = ut_x - hard_iron_bias_x;
  float ym_off = ut_y - hard_iron_bias_y;
  float zm_off = ut_z - hard_iron_bias_z;

  // 3. Soft-iron 3x3 correction
  LIS3MDL::vector<float> cal_m;
  cal_m.x = (xm_off * soft_iron_bias_xx) + (ym_off * soft_iron_bias_yx) + (zm_off * soft_iron_bias_zx);
  cal_m.y = (xm_off * soft_iron_bias_xy) + (ym_off * soft_iron_bias_yy) + (zm_off * soft_iron_bias_zy);
  cal_m.z = (xm_off * soft_iron_bias_xz) + (ym_off * soft_iron_bias_yz) + (zm_off * soft_iron_bias_zz);

  // 4. Gravity reference from the accelerometer
  LIS3MDL::vector<float> a = {(float)d.imu.a.x, (float)d.imu.a.y, (float)d.imu.a.z};

  // 5. East/North from cross products (tilt compensation)
  LIS3MDL::vector<float> E, N;
  LIS3MDL::vector_cross(&cal_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // 6. Project +X (forward) onto the horizontal plane.
  //    EMA the two projections BEFORE atan2 (not the heading itself).
  LIS3MDL::vector<float> from = {1, 0, 0};
  static EMA eFilter(0.2f), nFilter(0.2f);
  float eComp = eFilter.update(LIS3MDL::vector_dot(&E, &from));
  float nComp = nFilter.update(LIS3MDL::vector_dot(&N, &from));

  float heading = atan2f(eComp, nComp) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;
  return heading;
}

void readIMU(IMUData& d) {
  if (!d.ok) return;

  d.imu.read();
  d.mag.read();

  // Scale values
  d.accX = d.imu.a.x * (1.0f / 16384.0f);
  d.accY = d.imu.a.y * (1.0f / 16384.0f);
  d.accZ = d.imu.a.z * (1.0f / 16384.0f);

  // Gyro Z (yaw rate) in dps — LSM6 at ±2000 dps = 70 per dps
  d.gyroZ = d.imu.g.z / 70.0f;

  // Pitch and roll from accelerometer (body frame)
  float ax = d.imu.a.x, ay = d.imu.a.y, az = d.imu.a.z;
  float normA = sqrtf(ax * ax + ay * ay + az * az);
  if (normA < 1.0f) return;
  ax /= normA; ay /= normA; az /= normA;
  d.pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  d.roll  = atan2f(ay, az);

  // Heading from the tilt-compensated magnetometer, rotated into the grid
  // frame so 0° = up = (0,-1) (towards the back of the arena).
  d.headingDeg = rawToGridHeading(computeMagHeading(d));
}

// ── Bumper ──────────────────────────────────────────────────
// TODO: wire to Giga GPIO.

// ── Light Sensor ─────────────────────────────────────────────
// Phototransistor on analog A11 — analogRead only (not digital-capable).
// Higher value = more light (bonus points).

void initLightSensor() {
  // A8-A11 are analog-only — no pinMode needed, analogRead handles it.
}

int readLightSensor() {
  return analogRead(PIN_LIGHT_SENSOR);
}

#endif
