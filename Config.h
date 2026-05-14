#ifndef CONFIG_H
#define CONFIG_H

// --- Motor Control ---
// Motoron M3S550 on I2C Wire1, address 0x10
// Left = M1 (positive = forward), Right = M2 (negative = forward)

const int ENC_LEFT_A = 2;
const int ENC_LEFT_B = 3;
const int ENC_RIGHT_A = 4;
const int ENC_RIGHT_B = 5;

const float TICKS_PER_M = 5360.0;
const float TRACK_BASE_MM = 152.0; // distance between track centres

// --- Ultrasonic Sensors ---
// Mounting yaw: left = -32°, middle = 0°, right = +32° from forward
const int UDS_LEFT_TRIG = 32;
const int UDS_LEFT_ECHO = 33;
const int UDS_MID_TRIG = 34;
const int UDS_MID_ECHO = 35;
const int UDS_RIGHT_TRIG = 36;
const int UDS_RIGHT_ECHO = 37;

const float UDS_LEFT_YAW = -32.0;
const float UDS_MID_YAW = 0.0;
const float UDS_RIGHT_YAW = 32.0;

// --- IR Reflectance Array (9 sensors) ---
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};

// --- Servo (Seed Dispenser) ---
const int SERVO_PIN = 31;
const int SERVO_ANGLES[] = {0, 25, 65, 105, 145, 180};
const int SERVO_PLAY_DEG = 5; // TODO: measure mechanical play
const float SEED_DROP_X = 0.0; // mm from robot center, TODO: measure
const float SEED_DROP_Y = 0.0;

// --- Kill Switch / LED ---
const int KILL_BUTTON = 38;
const int ACT_LED = 39; // HIGH = circuit flashes red (dead/stopped)

// --- RFID ---
// MFRC522_I2C on Wire1, address 0x28

// --- IMU ---
// LIS3MDL (mag) + LSM6 (accel/gyro) on Wire

// --- WiFi ---
// UDP port 4210

// TODO: calibrate with Calibrate example
const int16_t MAG_MIN_X = -32767;
const int16_t MAG_MIN_Y = -32767;
const int16_t MAG_MIN_Z = -32767;
const int16_t MAG_MAX_X = 32767;
const int16_t MAG_MAX_Y = 32767;
const int16_t MAG_MAX_Z = 32767;

#endif
