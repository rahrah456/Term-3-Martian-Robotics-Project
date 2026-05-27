#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
// #include <MFRC522_I2C.h>
#include "Config.h"

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
    uint16_t v = 1000 - vals[i];
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
/*
#include <MFRC522_I2C.h>

bool readRFID(MFRC522_I2C& rfid, char* buf, size_t len) {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;
  size_t idx = 0;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10 && idx < len - 1) buf[idx++] = '0';
    int n = snprintf(&buf[idx], len - idx, "%X", rfid.uid.uidByte[i]);
    if (n <= 0) break;
    idx += n;
    if (i < rfid.uid.size - 1 && idx < len - 1) buf[idx++] = ' ';
    if (idx >= len - 1) break;
  }
  buf[idx] = '\0';
  return true;
}
*/

// ── IMU ─────────────────────────────────────────────────────
// LIS3MDL magnetometer + LSM6 accelerometer/gyro for
// tilt-compensated heading with gyro-assisted dead-reckoning.

struct IMUData {
  LIS3MDL mag;
  LSM6    imu;
  float headingDeg;     // tilt-compensated magnetometer heading
  float gyroZ;          // degrees per second (yaw rate)
  float magX, magY, magZ;
  float accX, accY, accZ;
  float pitch, roll;    // radians, for gravity subtraction
  bool  ok;
};

void initIMU(IMUData& d) {
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  d.ok = false;
  if (!d.mag.init())   { Serial.println("IMU: mag init failed"); return; }
  if (!d.imu.init())   { Serial.println("IMU: lsm6 init failed"); return; }
  d.mag.enableDefault();
  d.imu.enableDefault();
  d.ok = true;
  Serial.println("IMU: OK");
}

void readIMU(IMUData& d) {
  if (!d.ok) return;

  d.mag.read();
  d.imu.read();

  // Scale values
  d.magX = d.mag.m.x * (1.0f / 146.0f);
  d.magY = d.mag.m.y * (1.0f / 146.0f);
  d.magZ = d.mag.m.z * (1.0f / 146.0f);
  d.accX = d.imu.a.x * (1.0f / 16384.0f);
  d.accY = d.imu.a.y * (1.0f / 16384.0f);
  d.accZ = d.imu.a.z * (1.0f / 16384.0f);

  // Gyro Z (yaw rate) in dps — LSM6 at ±2000 dps = 70 per dps
  d.gyroZ = d.imu.g.z / 70.0f;

  // Hard-iron corrected magnetometer
  float mx = d.mag.m.x - ((int32_t)MAG_MIN_X + MAG_MAX_X) / 2.0f;
  float my = d.mag.m.y - ((int32_t)MAG_MIN_Y + MAG_MAX_Y) / 2.0f;
  float mz = d.mag.m.z - ((int32_t)MAG_MIN_Z + MAG_MAX_Z) / 2.0f;

  // Normalise accelerometer
  float ax = d.imu.a.x, ay = d.imu.a.y, az = d.imu.a.z;
  float normA = sqrtf(ax * ax + ay * ay + az * az);
  if (normA < 1.0f) return;
  ax /= normA; ay /= normA; az /= normA;

  // Pitch and roll from normalised accelerometer (body frame)
  d.pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  d.roll  = atan2f(ay, az);

  // East = cross(mag, gravity)
  float ex = ay * mz - az * my;
  float ey = az * mx - ax * mz;
  float ez = ax * my - ay * mx;
  float normE = sqrtf(ex * ex + ey * ey + ez * ez);
  if (normE < 1.0f) return;
  ex /= normE; ey /= normE; ez /= normE;

  // North = cross(gravity, east)
  float nx = ay * ez - az * ey;

  float h = atan2f(ex, nx) * 180.0f / PI;
  if (h < 0) h += 360.0f;
  d.headingDeg = h;
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
