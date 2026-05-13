// Trial 1 all-in-one demo
// Flash once, pick demos from the Serial menu

#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MFRC522_I2C.h>

// --- Pins ---
const int ENC_LA = 2, ENC_LB = 3, ENC_RA = 4, ENC_RB = 5;
const int UDS_LT = 32, UDS_LE = 33, UDS_MT = 34, UDS_ME = 35, UDS_RT = 36, UDS_RE = 37;
const uint8_t IR_COUNT = 9;
const uint8_t IR_PINS[] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
const int KILL_BTN = 38;
const int ACT_LED = 39;

// --- Hardware objects ---
MotoronI2C mc(0x10);
WiFiUDP udp;
MFRC522_I2C rfid(0x28, -1, &Wire1);

// --- Encoder state ---
volatile long encL = 0, encR = 0;

void isr_LA() { encL += (digitalRead(ENC_LA) == digitalRead(ENC_LB)) ? 1 : -1; }
void isr_LB() { encL += (digitalRead(ENC_LA) != digitalRead(ENC_LB)) ? 1 : -1; }
void isr_RA() { encR += (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ? 1 : -1; }
void isr_RB() { encR += (digitalRead(ENC_RA) != digitalRead(ENC_RB)) ? 1 : -1; }

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

void readIR(uint16_t vals[]) {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
  }
  delayMicroseconds(10);
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
    vals[i] = 1000;
  }
  unsigned long start = micros();
  while (micros() - start < 1000) {
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      if (vals[i] == 1000 && digitalRead(IR_PINS[i]) == LOW) {
        vals[i] = micros() - start;
      }
    }
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
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 200);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 200);
  setMotors(0, 0);

  // Kill switch / LED
  pinMode(KILL_BTN, INPUT_PULLUP);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);

  // Ultrasonic pins
  pinMode(UDS_LT, OUTPUT); pinMode(UDS_LE, INPUT);
  pinMode(UDS_MT, OUTPUT); pinMode(UDS_ME, INPUT);
  pinMode(UDS_RT, OUTPUT); pinMode(UDS_RE, INPUT);

  // RFID
  rfid.PCD_Init();

  Serial.println("Trial 1 ready");
  printMenu();
}

// ============================================================
//   MAIN LOOP
// ============================================================
void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case '1': demoMechanicalKill(); break;
    case '2': demoWiFiKill();       break;
    case '3': demoMotion();         break;
    case '4': demoIR();             break;
    case '5': demoUltrasonic();     break;
    case '6': demoRFID();           break;
    case '7': demoStreamAll();      break;
    case 'm': printMenu();          break;
  }
}

// ============================================================
//   DEMO 1: MECHANICAL KILL SWITCH
// ============================================================
void demoMechanicalKill() {
  Serial.println("\n--- Mechanical Kill Switch ---");
  Serial.println("Button toggles motors. 'x' to exit.");

  bool running = false;
  bool lastBtn = HIGH;
  unsigned long debounceTime = 0;

  while (true) {
    if (Serial.available() && Serial.read() == 'x') {
      setMotors(0, 0);
      digitalWrite(ACT_LED, LOW);
      return;
    }

    bool btn = digitalRead(KILL_BTN);
    if (btn == HIGH && lastBtn == LOW && millis() - debounceTime > 50) {
      debounceTime = millis();
      running = !running;
      if (running) {
        setMotors(400, 400);
        digitalWrite(ACT_LED, LOW);
        Serial.println("RUNNING");
      } else {
        setMotors(0, 0);
        digitalWrite(ACT_LED, HIGH);
        Serial.println("STOPPED - LED flashing red");
      }
    }
    lastBtn = btn;
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
//   DEMO 3: SPEED AND HEADING CONTROL
// ============================================================
void demoMotion() {
  Serial.println("\n--- Speed and Heading Control ---");
  Serial.println("Forward -> Backward -> Turn L -> Turn R -> U-turn");
  Serial.println("Place robot with space. Starting in 3s...");
  delay(3000);

  setMotors(400, 400);
  Serial.print("Forward 400"); printEncDelay(2000);

  setMotors(800, 800);
  Serial.print("Forward 800"); printEncDelay(2000);

  setMotors(-600, -600);
  Serial.print("Backward 600"); printEncDelay(2000);

  setMotors(-400, 400);
  Serial.print("Turn left 90"); printEncDelay(900);

  setMotors(400, -400);
  Serial.print("Turn right 90"); printEncDelay(900);

  setMotors(-400, 400);
  Serial.print("U-turn 180"); printEncDelay(1800);

  setMotors(0, 0);
  Serial.println("Motion demo done");
}

void printEncDelay(unsigned long ms) {
  delay(ms);
  setMotors(0, 0);
  Serial.print("  enc L:");
  Serial.print(encL);
  Serial.print(" R:");
  Serial.println(encR);
  delay(1000);
}

// ============================================================
//   DEMO 4: IR REFLECTANCE READOUT
// ============================================================
void demoIR() {
  Serial.println("\n--- IR Reflectance Array ---");
  Serial.println("S0\tS1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\t| Centroid");
  Serial.println("'x' to exit.");

  uint16_t vals[IR_COUNT];
  unsigned long lastHead = 0;

  while (true) {
    if (Serial.available() && Serial.read() == 'x') return;

    readIR(vals);
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      Serial.print(vals[i]);
      Serial.print("\t");
    }

    int pos = irCentroid(vals);
    if (pos >= 0) {
      Serial.print("| ");
      Serial.println(pos);
    } else {
      Serial.println("| no line");
    }

    if (millis() - lastHead > 5000) {
      lastHead = millis();
      Serial.println("S0\tS1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\t| Centroid");
    }
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
