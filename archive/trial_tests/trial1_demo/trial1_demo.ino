// Trial 1 all-in-one demo
// Flash once, pick demos from the Serial menu

#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MFRC522_I2C.h>
#include <Servo.h>


// --- Pins ---
const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
const int UDS_LT = 32, UDS_LE = 33, UDS_MT = 34, UDS_ME = 35, UDS_RT = 36, UDS_RE = 37;
const int KILL_BTN = 38;
const int ACT_LED = 39;
const int SERVO_PIN = 31;
// IR Sensor config
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {30, 23, 24, 25, 26, 27, 28, 29, 22};
const int IR_EMITTER_1 = 40;
const int IR_EMITTER_2 = 41;
const int TIMEOUT = 2500;


// --- Seed dispenser angles (0 = closed, 1-5 = seed positions) ---
const int SEED_ANGLES[] = {0, 25, 65, 105, 145, 180};
const int SEED_COUNT = 6;

// --- Robot constants ---
const float TICKS_PER_M = 5360.0;
const float TRACK_BASE_MM = 152.0;
const long TICKS_10CM = 0.10 * TICKS_PER_M;        // 536
const long TICKS_15CM = 0.15 * TICKS_PER_M;        // 804
const long TICKS_TURN_90 = PI * TRACK_BASE_MM * 90.0 / 360.0 * TICKS_PER_M / 1000.0 / 3.0 * 1.38;  // ~213*1.38
const long TICKS_TURN_180 = TICKS_TURN_90 * 2;      // ~427*1.38

// --- Hardware objects ---
MotoronI2C mc(0x10);
WiFiUDP udp;
MFRC522_I2C rfid(0x28, -1, &Wire1);
Servo servo;

// --- Global state ---
volatile long encL = 0, encR = 0;
bool killed = false;

void isr_LA() { encL += (digitalRead(ENC_LA) == digitalRead(ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(ENC_LA) != digitalRead(ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(ENC_RA) != digitalRead(ENC_RB)) ? 1 : -1; }

// --- E-Stop ---
bool handleEStop() {
  static bool lastBtn = HIGH;
  static unsigned long debounce = 0;

  bool btn = digitalRead(KILL_BTN);
  if (btn == HIGH && lastBtn == LOW && millis() - debounce > 50) {
    debounce = millis();
    killed = !killed;
    if (killed) {
      setMotors(0, 0);
      digitalWrite(ACT_LED, HIGH);
      Serial.println("E-STOP engaged");
    } else {
      digitalWrite(ACT_LED, LOW);
      Serial.println("E-STOP released");
    }
  }
  lastBtn = btn;

  return killed;
}

void waitForUnkill() {
  while (killed) {
    handleEStop();
    delay(20);
  }
}

// --- IMU (Adafruit libs) ---
#include <LIS3MDL.h>
#include <LSM6.h>

uint16_t myMins[9] = { 57, 31, 29, 26, 36, 44, 58, 81, 111 };
uint16_t myMaxs[9] = { 1577, 1173, 1067, 946, 946, 974, 998, 1173, 1501 };

LIS3MDL mag;
LSM6 imu;
float magHeading = 0.0;
float magGaussX = 0, magGaussY = 0, magGaussZ = 0;
float accGX = 0, accGY = 0, accGZ = 0;
bool imuOK = false;

const float MAG_SCALE = 1.0 / 146.0; // LSB to gauss at ±4 gauss
const float ACC_SCALE = 1.0 / 16384.0; // LSB to g at ±2g

// Calibration values — run the LIS3MDL Calibrate example, rotate the IMU in all
// directions for 30s, then replace these with the printed min/max values.
// This corrects for magnetic interference from motors/wiring (hard iron).
int16_t magMinX = -8808, magMinY = 3318, magMinZ = -521; // min: { -8808,  +3318,   -521}
int16_t magMaxX = -3540, magMaxY = 9119, magMaxZ = 5024; // max: { -3540,  +9119,  +5024}



void initIMU() {
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  if (!mag.init()) {
    Serial.println("IMU: mag init failed");
    return;
  }
  mag.enableDefault();

  if (!imu.init()) {
    Serial.println("IMU: lsm6 init failed");
    return;
  }
  imu.enableDefault();

  imuOK = true;
  Serial.println("IMU: OK");
}

void readIMU() {
  if (!imuOK) return;

  mag.read();
  imu.read();

  // Scaled values for display
  magGaussX = mag.m.x * MAG_SCALE;
  magGaussY = mag.m.y * MAG_SCALE;
  magGaussZ = mag.m.z * MAG_SCALE;
  accGX = imu.a.x * ACC_SCALE;
  accGY = imu.a.y * ACC_SCALE;
  accGZ = imu.a.z * ACC_SCALE;

  // Heading from raw values with hard-iron correction
  // Apply mounting orientation: swap which axis is "forward"
  float mx = mag.m.x - ((int32_t)magMinX + magMaxX) / 2.0;
  float my = mag.m.y - ((int32_t)magMinY + magMaxY) / 2.0;
  float mz = mag.m.z - ((int32_t)magMinZ + magMaxZ) / 2.0;

  // Accelerometer — normalize to get gravity direction
  float ax = imu.a.x, ay = imu.a.y, az = imu.a.z;
  float normA = sqrt(ax*ax + ay*ay + az*az);
  if (normA < 1) return;
  ax /= normA; ay /= normA; az /= normA;

  // East = cross(M, gravity), normalized
  float ex = ay*mz - az*my;
  float ey = az*mx - ax*mz;
  float ez = ax*my - ay*mx;
  float normE = sqrt(ex*ex + ey*ey + ez*ez);
  if (normE < 1) return;
  ex /= normE; ey /= normE; ez /= normE;

  // North = cross(gravity, East), normalized
  float nx = ay*ez - az*ey;
  float ny = az*ex - ax*ez;
  float nz = ax*ey - ay*ex;

  // Heading: angle between North and forward (+X) in horizontal plane
  float h = atan2(ex, nx) * 180 / PI;
  if (h < 0) h += 360;
  magHeading = h;
}

// --- Motor helper (M2 polarity already flipped) ---
void setMotors(int left, int right) {
  mc.setSpeed(1, left);
  mc.setSpeed(2, -right);
}

// --- Sensor helpers ---
long readUDS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 30000);
  return (us == 0) ? 999 : us / 58;
}

// Reads the raw microsecond discharge time of all 9 pins simultaneously
void readRawIR(uint16_t* rawVals) {
  // 1. Set all to OUTPUT and HIGH to charge the capacitors
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
    rawVals[i] = TIMEOUT; // Default to max timeout
  }
  
  delayMicroseconds(10); // Give them time to charge

  // 2. Switch all to INPUT to let them discharge
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  // 3. Watch all 9 pins simultaneously until they go LOW
  unsigned long startTime = micros();
  uint8_t pinsLeft = IR_COUNT;

  while (pinsLeft > 0) {
    unsigned long elapsed = micros() - startTime;
    if (elapsed >= TIMEOUT) break;

    for (uint8_t i = 0; i < IR_COUNT; i++) {
      // If this pin hasn't finished yet, check if it just went LOW
      if (rawVals[i] == TIMEOUT && digitalRead(IR_PINS[i]) == LOW) {
        rawVals[i] = elapsed;
        pinsLeft--;
      }
    }
  }
}

// Replaces the library's readCalibrated() function
void readIR(uint16_t vals[]) {
  uint16_t rawOff[IR_COUNT];
  uint16_t rawOn[IR_COUNT];

  // 1. Emitters are currently OFF. Take the ambient light reading.
  readRawIR(rawOff);

  // 2. Turn emitters ON and wait for them to power up
  digitalWrite(IR_EMITTER_1, HIGH);
  digitalWrite(IR_EMITTER_2, HIGH);
  delayMicroseconds(200); 

  // 3. Take the reading with emitters ON (ambient + reflected IR)
  readRawIR(rawOn);

  // 4. Turn emitters OFF to save power
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);

  // 5. Apply ambient light cancellation and map to 0-1000
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    // The official Pololu math for removing ambient light interference:
    int32_t adjustedRaw = (int32_t)rawOn[i] + TIMEOUT - (int32_t)rawOff[i];
    
    // Prevent underflow/overflow bounds
    if (adjustedRaw > TIMEOUT) adjustedRaw = TIMEOUT;
    if (adjustedRaw < 0) adjustedRaw = 0;

    // Map the adjusted raw value to your 0-1000 scale
    long mapped = map(adjustedRaw, myMins[i], myMaxs[i], 0, 1000);
    vals[i] = constrain(mapped, 0, 1000);
  }
}

int irCentroid(uint16_t vals[]) {
  uint32_t sum = 0, weighted = 0;
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    uint16_t v = 1000 - vals[i];
    if (v > 50) { sum += v; weighted += v * i * 1000; }
  }
  return (sum < 100) ? -1 : weighted / sum;
}

bool readRFID(char* buf, size_t len) {
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

// --- Menu ---
void printMenu() {
  Serial.println();
  Serial.println("=== Trial 1 Demo Menu ===");
  Serial.println("1 - Mechanical kill switch");
  Serial.println("2 - WiFi kill switch");
  Serial.println("3 - Speed and heading control");
  Serial.println("4 - IR reflectance readout");
  Serial.println("5 - Ultrasonic readout");
  Serial.println("6 - RFID readout");
  Serial.println("7 - Stream ALL sensors live");
  Serial.println("8 - Servo position control");
  Serial.println("9 - Speed/heading open loop");
  Serial.println("0 - IMU heading readout");
  Serial.println("s - Seed dispenser cycle");
  Serial.println("m - Show this menu");
  Serial.print("> ");
}

void waitForSerial(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial.available()) return;
  }
}

void clearSerial() {
  while (Serial.available()) Serial.read();
}

// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(1500);

  // Encoder ISRs
  pinMode(ENC_LA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LA), isr_LA, RISING);
  pinMode(ENC_LB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_LB), isr_LB, RISING);
  pinMode(ENC_RA, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RA), isr_RA, RISING);
  pinMode(ENC_RB, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_RB), isr_RB, RISING);

  // Motoron
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

  // Kill switch / LED
  pinMode(KILL_BTN, INPUT_PULLUP);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);

  // IR sensors
  pinMode(IR_EMITTER_1, OUTPUT);
  pinMode(IR_EMITTER_2, OUTPUT);
  digitalWrite(IR_EMITTER_1, LOW);
  digitalWrite(IR_EMITTER_2, LOW);

  // Ultrasonic pins
  pinMode(UDS_LT, OUTPUT); pinMode(UDS_LE, INPUT);
  pinMode(UDS_MT, OUTPUT); pinMode(UDS_ME, INPUT);
  pinMode(UDS_RT, OUTPUT); pinMode(UDS_RE, INPUT);

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(0);

  // IMU (init done inside initIMU)
  initIMU();

  // RFID
  rfid.PCD_Init();

  Serial.println("Trial 1 ready");
  printMenu();
}

// ============================================================
//   MAIN LOOP
// ============================================================
void loop() {
  handleEStop();

  if (!Serial.available()) {
    delay(20);
    return;
  }

  char c = Serial.read();

  switch (c) {
    case '1': demoMechanicalKill(); break;
    case '2': demoWiFiKill();       break;
    case '3': demoMotion();         break;
    case '4': demoIR();             break;
    case '5': demoUltrasonic();     break;
    case '6': demoRFID();           break;
    case '7': demoStreamAll();      break;
    case '8': demoServo();          break;
    case '9': demoOpenLoop();       break;
    case '0': demoIMU();            break;
    case 's': demoServoCycle();     break;
    case 'm': printMenu();          break;
  }
}

// ============================================================
//   DEMO 1: MECHANICAL KILL SWITCH
// ============================================================
void demoMechanicalKill() {
  Serial.println("\n--- Mechanical Kill Switch ---");
  Serial.println("Button toggles motors. 'x' to exit.");

  killed = false;
  digitalWrite(ACT_LED, LOW);

  while (true) {
    if (Serial.available() && Serial.read() == 'x') {
      setMotors(0, 0);
      digitalWrite(ACT_LED, LOW);
      return;
    }

    if (handleEStop()) { setMotors(0, 0); waitForUnkill(); continue; }
    setMotors(400, 400);
    delay(20);
  }
}

// ============================================================
//   DEMO 2: WIFI KILL SWITCH
// ============================================================
void demoWiFiKill() {
  Serial.println("\n--- WiFi Kill Switch ---");
  Serial.println("Connecting to WiFi...");

  const char* ssid = "wip13";
  const char* pass = "123456789";

  WiFi.begin(ssid, pass);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed - check network and restart demo");
    return;
  }

  Serial.println("\nConnected");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(4210);

  setMotors(300, 300);
  digitalWrite(ACT_LED, LOW);
  Serial.println("Motors running. Send 'Stop' / 'Go' via UDP. 'x' to exit.");

  char buf[255];

  while (true) {
    if (handleEStop()) { setMotors(0, 0); waitForUnkill(); setMotors(300, 300); continue; }

    if (Serial.available() && Serial.read() == 'x') {
      setMotors(0, 0);
      digitalWrite(ACT_LED, LOW);
      return;
    }

    int size = udp.parsePacket();
    if (size) {
      int len = udp.read(buf, 255);
      if (len > 0) buf[len] = 0;
      String msg = String(buf);
      IPAddress sender = udp.remoteIP();
      int port = udp.remotePort();

      Serial.print("Got: ");
      Serial.println(msg);

      if (msg == "Stop") {
        setMotors(0, 0);
        digitalWrite(ACT_LED, HIGH);
        udp.beginPacket(sender, port);
        udp.print("Stopped");
        udp.endPacket();
        Serial.println("KILLED");
      } else if (msg == "Go") {
        setMotors(300, 300);
        digitalWrite(ACT_LED, LOW);
        udp.beginPacket(sender, port);
        udp.print("Running");
        udp.endPacket();
        Serial.println("ALIVE");
      } else {
        udp.beginPacket(sender, port);
        udp.print("Unknown: " + msg);
        udp.endPacket();
      }
    }
    delay(10);
  }
}

// ============================================================
//   PID SPEED CONTROLLER
// ============================================================
struct PIDSpeed {
  float integral, prevErr, kp, ki, kd;
  unsigned long lastMicros;
  long lastEnc;

  PIDSpeed(float p, float i, float d) : integral(0), prevErr(0), kp(p), ki(i), kd(d), lastMicros(0), lastEnc(0) {}

  void reset(long encNow) {
    integral = 0; prevErr = 0; lastEnc = encNow; lastMicros = micros();
  }

  int update(long encNow, int target) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    if (dt <= 0 || dt > 0.05) dt = 0.02;
    float actual = (encNow - lastEnc) / dt;
    lastEnc = encNow;
    lastMicros = now;

    float err = target - actual;
    integral += err * dt;
    integral = constrain(integral, -400, 400);
    float deriv = (err - prevErr) / dt;
    prevErr = err;
    return constrain(kp * err + ki * integral + kd * deriv, -800, 800);
  }
};

PIDSpeed pidL(0.8, 0.15, 0.05);
PIDSpeed pidR(0.8, 0.15, 0.05);

void moveStraight(int speed, long ticks) {
  pidL.reset(encL);
  pidR.reset(encR);
  long startL = encL, startR = encR;
  unsigned long timeout = millis() + 10000;
  bool timedOut = false;

  while (true) {
    if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
    if (millis() >= timeout) { timedOut = true; break; }

    int cmdL = pidL.update(encL, speed);
    int cmdR = pidR.update(encR, speed);
    setMotors(cmdL, cmdR);

    if (abs(encL - startL) >= ticks && abs(encR - startR) >= ticks) break;
    delay(20);
  }
  setMotors(0, 0);
  long mL = abs(encL - startL);
  long mR = abs(encR - startR);
  Serial.print("  L:"); Serial.print(mL); Serial.print("/"); Serial.print(ticks);
  Serial.print("  R:"); Serial.print(mR); Serial.print("/"); Serial.print(ticks);
  if (timedOut) Serial.print("  TIMEOUT");
  else if (killed) Serial.print("  E-STOP");
  Serial.println();
}

void turn(int dir, int speed, long ticks) {
  pidL.reset(encL);
  pidR.reset(encR);
  long startL = encL, startR = encR;
  unsigned long timeout = millis() + 10000;
  bool timedOut = false;

  while (true) {
    if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
    if (millis() >= timeout) { timedOut = true; break; }

    int cmdL = pidL.update(encL, dir * speed);
    int cmdR = pidR.update(encR, -dir * speed);
    setMotors(cmdL, cmdR);

    if (abs(encL - startL) >= ticks && abs(encR - startR) >= ticks) break;
    delay(20);
  }
  setMotors(0, 0);
  long mL = abs(encL - startL);
  long mR = abs(encR - startR);
  Serial.print("  L:"); Serial.print(mL); Serial.print("/"); Serial.print(ticks);
  Serial.print("  R:"); Serial.print(mR); Serial.print("/"); Serial.print(ticks);
  if (timedOut) Serial.print("  TIMEOUT");
  else if (killed) Serial.print("  E-STOP");
  Serial.println();
}

// ============================================================
//   DEMO 3: SPEED AND HEADING CONTROL
// ============================================================
void demoMotion() {
  Serial.println("\n--- Speed and Heading Control ---");
  Serial.println("Forward 15cm -> Back 15cm -> Turn L 90 -> Turn R 90 -> U-turn");
  Serial.println("Ticks/m: 5360  Track base: 152mm");
  Serial.println("Place robot with space. Starting in 3s...");
  delay(3000);

  encL = 0; encR = 0;

  Serial.print("Forward 15cm");
  moveStraight(800, TICKS_15CM);
  if (killed) return;
  delay(1000);

  Serial.print("Backward 15cm");
  moveStraight(-800, TICKS_15CM);
  if (killed) return;
  delay(1000);

  Serial.print("Turn left 90");
  turn(1, 800, TICKS_TURN_90);
  if (killed) return;
  delay(1000);

  Serial.print("Turn right 90");
  turn(-1, 800, TICKS_TURN_90);
  if (killed) return;
  delay(1000);

  Serial.print("U-turn 180");
  turn(1, 800, TICKS_TURN_180);
  if (killed) return;
  delay(1000);

  Serial.println("Motion demo done");
}

// ============================================================
//   DEMO 4: IR REFLECTANCE READOUT
// ============================================================
void demoIR() {
  Serial.println("\n--- IR Reflectance Array ---");
  Serial.println("S0\tS1\tS2\tS3\tS4\tS5\tS6\tS7\tS8");
  // Serial.println("'x' to exit.");

  uint16_t vals[IR_COUNT] = {0};

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }
    if (Serial.available() && Serial.read() == 'x') return;

    readIR(vals);
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      Serial.print(vals[i]);
      Serial.print("\t");
    }
    Serial.println();
    delay(100);
  }
}

// ============================================================
//   DEMO 5: ULTRASONIC READOUT
// ============================================================
void demoUltrasonic() {
  Serial.println("\n--- Ultrasonic Distance (cm) ---");
  Serial.println("Left(-32)\tMid(0)\t\tRight(+32)\t");
  Serial.println("'x' to exit.");

  unsigned long lastHead = 0;

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }
    if (Serial.available() && Serial.read() == 'x') return;

    long l = readUDS(UDS_LT, UDS_LE);
    long m = readUDS(UDS_MT, UDS_ME);
    long r = readUDS(UDS_RT, UDS_RE);

    Serial.print(l < 999 ? l : 0); Serial.print("\t\t");
    Serial.print(m < 999 ? m : 0); Serial.print("\t\t");
    Serial.println(r < 999 ? r : 0);

    if (millis() - lastHead > 5000) {
      lastHead = millis();
      Serial.println("Left(-32)\tMid(0)\t\tRight(+32)\t");
    }
    delay(200);
  }
}

// ============================================================
//   DEMO 7: STREAM ALL SENSORS
// ============================================================
void demoStreamAll() {
  Serial.println("\n--- All Sensors Live ---");
  Serial.println("IR(S0-S8 | C)    \tUDS(L M R cm)\tRFID");
  Serial.println("'x' to exit.");
  Serial.println();

  uint16_t ir[IR_COUNT];
  char uid[32];
  unsigned long lastHead = 0;

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }
    if (Serial.available() && Serial.read() == 'x') return;

    // IR
    readIR(ir);
    int pos = irCentroid(ir);
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      Serial.print(ir[i]); Serial.print(" ");
    }
    if (pos >= 0) { Serial.print("| "); Serial.print(pos); }
    else { Serial.print("| ---"); }
    Serial.print("\t");

    // UDS
    long l = readUDS(UDS_LT, UDS_LE);
    long m = readUDS(UDS_MT, UDS_ME);
    long r = readUDS(UDS_RT, UDS_RE);
    Serial.print("UDS: "); Serial.print(l < 999 ? l : 0);
    Serial.print(" "); Serial.print(m < 999 ? m : 0);
    Serial.print(" "); Serial.print(r < 999 ? r : 0);
    Serial.print("\t");

    // RFID
    if (readRFID(uid, sizeof(uid))) {
      Serial.print("RFID: ");
      Serial.println(uid);
    } else {
      Serial.println("RFID: ---");
    }

    if (millis() - lastHead > 5000) {
      lastHead = millis();
      Serial.println("--- IR(S0-S8 | C)    \tUDS(L M R cm)\tRFID ---");
    }
    delay(200);
  }
}

// ============================================================
//   DEMO 6: RFID READOUT
// ============================================================
void demoRFID() {
  Serial.println("\n--- RFID Reader ---");
  Serial.println("Tap a tag to read. 'x' to exit.");

  char uid[32];
  unsigned long lastMsg = 0;

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }
    if (Serial.available() && Serial.read() == 'x') return;

    if (readRFID(uid, sizeof(uid))) {
      Serial.print("Tag detected: ");
      Serial.println(uid);
      lastMsg = millis();
    }

    if (millis() - lastMsg > 2000) {
      Serial.println("Waiting for tag...");
      lastMsg = millis();
    }
    delay(200);
  }
}

// ============================================================
//   DEMO 8: SERVO POSITION CONTROL
// ============================================================
void demoServo() {
  Serial.println("\n--- Servo Position Control ---");
  Serial.println("Enter an angle (0-270) to move the servo.");
  Serial.println("'x' to exit.");

  servo.write(0);

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }

    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line == "x") return;

      int angle = line.toInt();
      if (angle >= 0 && angle <= 270) {
        servo.write(angle);
        Serial.print("Moved to ");
        Serial.println(angle);
      } else {
        Serial.println("Enter 0-180");
      }
    }
    delay(20);
  }
}

// ============================================================
//   DEMO 9: SPEED/HEADING OPEN LOOP
// ============================================================
void demoOpenLoop() {
  Serial.println("\n--- Speed/Heading Open Loop ---");
  // Serial.println("Fwd 800 -> Fwd 600 -> Turn L -> Turn R -> U-turn");
  Serial.println("Place robot with space. Starting in 3s...");
  delay(3000);

  auto runOL = [&](int left, int right, long targetTicks, const char* label) {
    encL = 0; encR = 0;
    unsigned long timeout = millis() + 10000;
    bool timedOut = false;
    setMotors(left, right);

    while (true) {
      if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
      if (millis() >= timeout) { timedOut = true; break; }
      if (abs(encL) >= targetTicks && abs(encR) >= targetTicks) break;
      delay(20);
    }
    setMotors(0, 0);

    Serial.print(label);
    Serial.print("  L:"); Serial.print(abs(encL)); Serial.print("/"); Serial.print(targetTicks);
    Serial.print("  R:"); Serial.print(abs(encR)); Serial.print("/"); Serial.print(targetTicks);
    if (timedOut) Serial.print("  TIMEOUT");
    else if (killed) Serial.print("  E-STOP");
    Serial.println();
  };

  runOL(660, 660, TICKS_10CM*100, "Fwd 660");
  if (killed) return;
  delay(500);
  // runOL(-660, -660, TICKS_10CM, "Back 660");
  // if (killed) return;
  // delay(500);
  // runOL(600, 600, TICKS_10CM, "Fwd 600");
  // if (killed) return;
  // delay(500);
  // runOL(-660, 660, TICKS_TURN_90, "Turn left");
  // if (killed) return;
  // delay(500);
  // runOL(660, -660, TICKS_TURN_90, "Turn right");
  // if (killed) return;
  // delay(500);
  // runOL(-660, 660, TICKS_TURN_180, "U-turn");

  if (!killed) Serial.println("Open loop demo done");
}

// ============================================================
//   SEED DISPENSER CYCLE
// ============================================================
void demoServoCycle() {
  Serial.println("\n--- Seed Dispenser ---");
  Serial.println("Enter position 0-5 to move servo.");
  Serial.println("  0 = closed");
  Serial.println("  1-5 = seed positions");
  Serial.println("'x' to exit.");

  servo.write(0);

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }

    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line == "x") return;

      int pos = line.toInt();
      if (pos >= 0 && pos < SEED_COUNT) {
        servo.write(SEED_ANGLES[pos]);
        Serial.print("Position "); Serial.print(pos);
        Serial.print(" ("); Serial.print(SEED_ANGLES[pos]); Serial.println("deg)");
      } else {
        Serial.print("Enter 0-"); Serial.println(SEED_COUNT - 1);
      }
    }
    delay(20);
  }
}

// ============================================================
//   DEMO 0: IMU HEADING READOUT
// ============================================================
void demoIMU() {
  Serial.println("\n--- IMU Heading ---");

  if (!imuOK) {
    Serial.println("IMU not initialised — check wiring and restart");
    return;
  }

  Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tMagX\tMagY\tMagZ\tHeading(deg)");
  Serial.println("'x' to exit.");

  auto pf = [](float v, int w, int d) {
    char b[16]; sprintf(b, "%*.*f", w, d, v); Serial.print(b);
  };
  unsigned long lastHead = 0;

  while (true) {
    if (handleEStop()) { waitForUnkill(); continue; }
    if (Serial.available() && Serial.read() == 'x') return;

    readIMU();

    pf(accGX, 7, 3); pf(accGY, 7, 3); pf(accGZ, 7, 3);
    pf(magGaussX, 6, 1); pf(magGaussY, 6, 1); pf(magGaussZ, 6, 1);
    pf(magHeading, 6, 1); Serial.println();

    if (millis() - lastHead > 5000) {
      lastHead = millis();
      Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tMagX\tMagY\tMagZ\tHeading(deg)");
    }
    delay(200);
  }
}
