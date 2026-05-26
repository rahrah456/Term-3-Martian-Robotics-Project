#include <Wire.h>
#include <Motoron.h>

// --- Pins ---
const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
const int UDS_MID_TRIG = 34;
const int UDS_MID_ECHO = 35;
const int KILL_BTN = 38;

// --- Globals ---
volatile long encL = 0, encR = 0;
bool killed = false;
MotoronI2C mc(0x10);
const int MAX_MOTOR_POWER = 660;

// --- Encoder ISRs ---
void isr_LA() { encL += (digitalRead(ENC_LA) == digitalRead(ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(ENC_LA) != digitalRead(ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(ENC_RA) != digitalRead(ENC_RB)) ? 1 : -1; }

// --- Motor helper ---
void setMotors(int left, int right) {
  mc.setSpeed(1, left);
  mc.setSpeed(2, -right);
}

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
      Serial.println("E-STOP engaged");
    } else {
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

// --- PID speed controller ---
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

// --- Ultrasonic read ---
long readUDS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 30000);
  return (us == 0) ? 999 : us / 58;
}

// Default UDS read (middle sensor)
long readUDS() { return readUDS(UDS_MID_TRIG, UDS_MID_ECHO); }

// ============================================================
//   WALL FOLLOWING  —  cascaded PID
//   Sensor mounted at 58° to wall (right side assumed)
//   d_perp = d_measured * cos(58°) = d_measured * 0.5299
// ============================================================

// Outer loop: wall distance PID
// Tune these independently from PIDSpeed gains
float wKp = 1.5f, wKi = 0.0f, wKd = 0.0f;
float wIntegral   = 0.0f;
float wPrevError  = 0.0f;
unsigned long wPrevTime = 0;

const float WALL_SETPOINT_CM = 20.0f;   // desired perp distance
const float COS58 = 0.5299f;
const float MAX_CORRECTION = 300.0f;    // clamp — tune this
const float WINDUP_LIMIT   = 150.0f;

float wallPID(float dMeasured) {
    float dPerp = dMeasured * COS58;
    float error = dPerp - WALL_SETPOINT_CM;

    unsigned long now = millis();
    float dt = (now - wPrevTime) / 1000.0f;
    wPrevTime = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;  // guard bad dt on first call

    // Integral with anti-windup
    if (abs(error) < 10.0f)
        wIntegral += error * dt;
    wIntegral = constrain(wIntegral, -WINDUP_LIMIT, WINDUP_LIMIT);

    float derivative = (error - wPrevError) / dt;
    wPrevError = error;

    float correction = wKp*error + wKi*wIntegral + wKd*derivative;
    return constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
}

// Median filter — rejects UDS spikes from oblique angle
float medianOf5(float a,float b,float c,float d,float e){
    float v[5]={a,b,c,d,e};
    // simple insertion sort on 5 elements
    for(int i=1;i<5;i++){
        float key=v[i]; int j=i-1;
        while(j>=0 && v[j]>key){ v[j+1]=v[j]; j--; }
        v[j+1]=key;
    }
    return v[2];
}

void wallFollow(int baseSpeed) {
    pidL.reset(encL);
    pidR.reset(encR);
    wIntegral = 0; wPrevError = 0;
    wPrevTime = millis();

    float buf[5] = {0}; int bufIdx = 0; bool bufFull = false;

    Serial.println("Wall following — send any char to stop");

    while (true) {
        if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
        if (Serial.available())  { setMotors(0, 0); break; }

        // --- Sensor read with median filter ---
        buf[bufIdx] = readUDS();            // your existing UDS read
        bufIdx = (bufIdx + 1) % 5;
        if (bufIdx == 0) bufFull = true;

        float dist;
        if (bufFull)
            dist = medianOf5(buf[0],buf[1],buf[2],buf[3],buf[4]);
        else
            dist = buf[max(0, bufIdx-1)];   // use latest until buffer fills

        // --- Outer wall PID ---
        float correction = wallPID(dist);

        // correction > 0 → too far from wall → steer right (toward wall)
        // correction < 0 → too close         → steer left  (away from wall)
        // Assumes sensor is on the RIGHT side; swap signs if on left
        int speedL = baseSpeed + (int)correction;
        int speedR = baseSpeed - (int)correction;

        // Prevent either tread reversing unintentionally
        speedL = constrain(speedL, 0, 1023);
        speedR = constrain(speedR, 0, 1023);

        // --- Inner velocity PIDs (unchanged) ---
        int cmdL = pidL.update(encL, speedL);
        int cmdR = pidR.update(encR, speedR);
        setMotors(cmdL, cmdR);

        // --- Debug ---
        Serial.print("dRaw:"); Serial.print(dist,1);
        Serial.print(" dPerp:"); Serial.print(dist*COS58,1);
        Serial.print(" err:"); Serial.print(dist*COS58 - WALL_SETPOINT_CM,1);
        Serial.print(" corr:"); Serial.print(correction,1);
        Serial.print(" L:"); Serial.print(speedL);
        Serial.print(" R:"); Serial.println(speedR);

        delay(20);   // 50 Hz — matches your existing loop rate
    }
    setMotors(0, 0);
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

  pinMode(UDS_MID_TRIG, OUTPUT);
  pinMode(UDS_MID_ECHO, INPUT);
  pinMode(KILL_BTN, INPUT_PULLUP);

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

  Serial.println("Wall Following PID ready");
  Serial.print("Enter base speed (0-1023): ");
}

// ============================================================
//   LOOP
// ============================================================
void loop() {
  if (Serial.available()) {
    int speed = Serial.parseInt();
    // consume trailing newline so wallFollow() doesn't bail immediately
    while (Serial.available()) Serial.read();
    if (speed > 0 && speed <= 1023) {
      wallFollow(speed);
    }
    Serial.print("Enter base speed (0-1023): ");
  }
  delay(20);
}