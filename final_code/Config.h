#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  CONFIG  —  pin assignments, robot constants, calibration
//  All measurements in mm / degrees unless stated.
// ============================================================

// ── Motor Control ───────────────────────────────────────────
// Motoron M3S550 on I2C Wire1, address 0x10
// M1 = right track, M2 = left track (physically swapped).
// M2 polarity reversed (negative = forward).
const int PIN_ENC_LA = 4;   // left track encoder
const int PIN_ENC_LB = 5;
const int PIN_ENC_RA = 2;   // right track encoder
const int PIN_ENC_RB = 3;

const float TRACK_BASE_MM = 161.0;  // distance between tread centres
const int MOTOR_MIN = 300;           // below this, friction wins
const int MOTOR_MAX = 660;           // absolute ceiling
const int STEERING_MAX_DIFF = 80;    // max differential for line/wall follow

// ── IR Reflectance Array (9 sensors) ────────────────────────
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {30, 23, 24, 25, 26, 27, 28, 29, 22};
const int IR_EMITTER_1 = 40;
const int IR_EMITTER_2 = 41;
const int IR_TIMEOUT_US = 2500;

// Calibrated min/max from trial1_demo
const uint16_t IR_MINS[9] = { 57, 31, 29, 26, 36, 44, 58, 81, 111 };
const uint16_t IR_MAXS[9] = { 1577, 1173, 1067, 946, 946, 974, 998, 1173, 1501 };

// ── Component Positions (mm from robot centre, +X=right, +Y=forward) ──
const float POS_LIGHT_SENSOR_X = 45.0f,  POS_LIGHT_SENSOR_Y = 68.0f;
const float POS_UDS_SIDE_X = 66.0f,       POS_UDS_SIDE_Y = -50.0f;
const float POS_SEED_CHUTE_X = 0.0f,      POS_SEED_CHUTE_Y = -30.0f;
const float POS_RFID_ANTENNA_X = 0.0f,    POS_RFID_ANTENNA_Y = -10.0f;
const float POS_IR_CENTRE_X = 0.0f,       POS_IR_CENTRE_Y = 17.0f;

// ── RFID Detection Range (mm) ───────────────────────────────
const float RFID_RANGE_TOP_BOTTOM = 40.0f;  // approached from top/bottom
const float RFID_RANGE_SIDE = 25.0f;        // approached from left/right
const float RFID_RANGE_DEVIATION = 5.0f;    // card placement deviation

// ── Robot Dimensions (mm) ───────────────────────────────────
const float CHASSIS_WIDTH = 115.0f;
const float CHASSIS_LENGTH = 170.0f;
const float TREAD_WIDTH = 27.0f;

// ── Ultrasonic Sensors ──────────────────────────────────────
// Mounting yaw: left = -32°, middle = 0°, right = +32° from forward
const int PIN_UDS_LT = 32, PIN_UDS_LE = 33;
const int PIN_UDS_MT = 34, PIN_UDS_ME = 35;
const int PIN_UDS_RT = 36, PIN_UDS_RE = 37;

const float UDS_LEFT_YAW = -32.0;
const float UDS_MID_YAW   =  0.0;
const float UDS_RIGHT_YAW = 32.0;
const float UDS_CONE_DEG  = 20.0;   // half-angle of detection cone
const int   UDS_TIMEOUT_US = 8000;    // pulseIn timeout per sensor (µs, ≈138cm)
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
// Hard-iron calibration (recalibrated)
const int16_t MAG_MIN_X = -7832; // -7078;
const int16_t MAG_MIN_Y =  1548; // 2593;
const int16_t MAG_MIN_Z = -3134; // 91;
const int16_t MAG_MAX_X = -2232; // -3234;
const int16_t MAG_MAX_Y =  5844; // 6347;
const int16_t MAG_MAX_Z =  4977; // 4698;

// ── Bumper (TODO: confirm pin once wired) ───────────────────
// Digital switch on front bumper — TODO: wire to Giga GPIO
// const int PIN_BUMPER = xx;

// ── Light Sensor ─────────────────────────────────────────────
// Phototransistor on Giga analog pin A11 (bonus points)
#define PIN_LIGHT_SENSOR A11

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

// ── Deposit Sequence ────────────────────────────────────────
const float DEPOSIT_HALF_DETECT_MM = 12.5f;       // half of lateral RFID range
const float DEPOSIT_RFID_TO_CHUTE_MM = 20.0f;     // 2cm from RFID antenna to chute
const float DEPOSIT_EXTRA_MM = DEPOSIT_HALF_DETECT_MM + DEPOSIT_RFID_TO_CHUTE_MM;

// ── Navigation Speeds ───────────────────────────────────────
const int MOVE_SPEED = 500;    // constant speed for straight-line moves
const int TURN_SPEED = 660;    // constant speed for tank turns

// ── Motor Bias ──────────────────────────────────────────────
// Left track is ~4.2% stronger; compensate by giving right track more power.
const float BIAS_RIGHT = 1.047f;

// ── Tick Calibration ──────────────────────────────────────────
// Fitted from tick_calibration data:
//   distance_mm = 4.44137 * log(ticks+1) + 0.273899 * ticks
//   turn ticks  = 6.0061 * theta_degrees

static inline long ticksForDistance(float mm) {
  bool neg = mm < 0;
  float a = neg ? -mm : mm;
  float t = 34.21488f * expf(-0.0870233f * a - 1.0f) + 3.60454f * a;
  long r = (long)(t > 0 ? t : 0);
  return neg ? -r : r;
}
static inline long ticksForTurn(float degrees) {
  return (long)(6.0061f * degrees);
}
static inline float ticksToMm(long ticks) {
  bool neg = ticks < 0;
  long a = neg ? -ticks : ticks;
  float r = 4.44137f * logf((float)(a + 1)) + 0.273899f * (float)a;
  return neg ? -r : r;
}

#endif
