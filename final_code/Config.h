#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  CONFIG  —  pin assignments, robot constants, calibration
//  All measurements in mm / degrees unless stated.
// ============================================================

// ── Motor Control ───────────────────────────────────────────
// Motoron M3S550 on I2C Wire1, address 0x10
// Left = M1 (positive = forward), Right = M2 (negative = forward)
const int PIN_ENC_LA = 2;
const int PIN_ENC_LB = 3;
const int PIN_ENC_RA = 4;
const int PIN_ENC_RB = 5;

const float TICKS_PER_M = 5360.0;   // TODO: calibrate on arena
const float TRACK_BASE_MM = 152.0;  // distance between tread centres
const int MOTOR_MIN = 300;           // below this, friction wins
const int MOTOR_MAX = 660;           // absolute ceiling
const int STEERING_MAX_DIFF = 40;    // max differential for line/wall follow

// ── IR Reflectance Array (9 sensors) ────────────────────────
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {30, 23, 24, 25, 26, 27, 28, 29, 22};
const int IR_EMITTER_1 = 40;
const int IR_EMITTER_2 = 41;
const int IR_TIMEOUT_US = 2500;

// Calibrated min/max from trial1_demo
const uint16_t IR_MINS[9] = { 57, 31, 29, 26, 36, 44, 58, 81, 111 };
const uint16_t IR_MAXS[9] = { 1577, 1173, 1067, 946, 946, 974, 998, 1173, 1501 };

// ── Ultrasonic Sensors ──────────────────────────────────────
// Mounting yaw: left = -32°, middle = 0°, right = +32° from forward
const int PIN_UDS_LT = 32, PIN_UDS_LE = 33;
const int PIN_UDS_MT = 34, PIN_UDS_ME = 35;
const int PIN_UDS_RT = 36, PIN_UDS_RE = 37;

const float UDS_LEFT_YAW = -32.0;
const float UDS_MID_YAW   =  0.0;
const float UDS_RIGHT_YAW = 32.0;
const float UDS_CONE_DEG  = 20.0;   // half-angle of detection cone
const int   UDS_TIMEOUT_US = 30000;
const int   UDS_MAX_CM     = 500;

// Stopping distance for obstacle avoidance (mm)
const int OBSTACLE_STOP_MM = 300;

// ── Servo (Seed Dispenser) ──────────────────────────────────
const int PIN_SERVO = 31;
// 5 seeds: positions 1-5. Angle 0 = locked (all seeds retained).
const int SEED_ANGLES[] = {0, 25, 65, 105, 145, 180};
const int SEED_COUNT = 6;    // index 0 = locked, 1-5 = dispense

// ── Kill Switch / LED ───────────────────────────────────────
const int PIN_KILL_BTN = 38;
const int PIN_ACT_LED  = 39;    // HIGH = red (stopped), LOW = green (running)

// ── RFID ────────────────────────────────────────────────────
// MFRC522_I2C on Wire1, address 0x28

// ── IMU ─────────────────────────────────────────────────────
// LIS3MDL (mag) + LSM6 (accel/gyro) on Wire
// Hard-iron calibration from trial1_demo
const int16_t MAG_MIN_X = -8808;
const int16_t MAG_MIN_Y =  3318;
const int16_t MAG_MIN_Z =  -521;
const int16_t MAG_MAX_X = -3540;
const int16_t MAG_MAX_Y =  9119;
const int16_t MAG_MAX_Z =  5024;

// ── Bumper (TODO: confirm pin once wired) ───────────────────
// Digital switch on front bumper — TODO: wire to Giga GPIO
// const int PIN_BUMPER = xx;

// ── Light Sensor (TODO: bonus points) ───────────────────────
// const int PIN_LIGHT = xx;

// ── Arena Constants ─────────────────────────────────────────
const float ARENA_SIZE_MM = 2500.0;
const int   GRID_HOLES    = 9;           // 9×9 grid
const float HOLE_SPACING_MM = 250.0;
const float HOLE_DIAMETER_MM = 22.0;

// Offset from base origin to arena corner, in mm
// TODO: measure once arena is known
const float BASE_TO_ARENA_X = 0.0;
const float BASE_TO_ARENA_Y = 0.0;

// ── Motor Acceleration / Deceleration ───────────────────────
const int MOTOR_RAMP = 800;   // Motoron max accel/decel

#endif
