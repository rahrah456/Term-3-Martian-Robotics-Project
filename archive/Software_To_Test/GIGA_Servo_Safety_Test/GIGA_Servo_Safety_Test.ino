#include <MiniMessenger.h>
#include <Servo.h>
#include "secrets.h" 

/*
 * GIGA Servo Safety Test
 * 
 * This sketch demonstrates the "Bare Minimum" safety integration for an Arduino GIGA.
 * - Registers with the server every 10 seconds.
 * - Listens for Emergency Stop and Individual Disable commands.
 * - Only moves the Servo when the server sends a "Heartbeat" (enable=1).
 */

MiniMessenger messenger;
Servo myServo;

// --- CONFIGURATION ---
const int SERVO_PIN = 9;         // Signal wire to Pin 9
const char* BoardId = "Unknown";     // Your Board ID (must match dashboard)
bool safetyEnabled = false;      // Controlled by Heartbeats/Emergency commands

// --- STATE VARIABLES ---
unsigned long lastRegisterMs = 0;
int servoPos = 0;
int sweepDirection = 1;
unsigned long lastMoveMs = 0;

// --- SAFETY WATCHDOG ---
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000; // 4x the 250ms server interval

// This function runs every time an MQTT message arrives for this robot
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[128];
    size_t copyLen = (length < 127) ? length : 127;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';

    // 1. Heartbeat Check
    if (strstr(msg, "type=heartbeat")) {
        lastHeartbeatMs = millis(); // Track that server is alive

        if (strstr(msg, "enable=1")) {
            safetyEnabled = true;
        } else if (strstr(msg, "enable=0")) {
            safetyEnabled = false;
            Serial.println("SAFETY: Heartbeat Disabled");
        }
    }

    // 2. Emergency/Disable Check (Immediate Stop)
    if (strstr(msg, "type=emergency enabled=true") || 
        strstr(msg, "type=disable enabled=false")) {
        safetyEnabled = false;
        Serial.println("!!! EMERGENCY/DISABLE ACTIVE !!!");
    }
}

void setup() {
    Serial.begin(115200);
        
    Serial.println("\n--- GIGA SERVO SAFETY TEST STARTING ---");

    myServo.attach(SERVO_PIN);
    myServo.write(90); // Start at neutral position

    // Initialize Network and MQTT
    messenger.onMessage(onMessage);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
    
    Serial.println("Network Connecting...");
}

void loop() {
    // Required to process incoming MQTT messages
    messenger.loop();

    // --- HEARTBEAT WATCHDOG ---
    // If we were enabled but haven't heard from the server in a while, disable.
    if (safetyEnabled && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
        safetyEnabled = false;
        Serial.println("SAFETY: Heartbeat Timeout (Server Connection Lost)");
    }

    // 1. REGISTRATION: Tell the server we are here every 10 seconds
    if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
        lastRegisterMs = millis();
        char reg[64];
        snprintf(reg,
                 sizeof(reg), 
                 "type=register team_id=%s board_id=%s",
                 GROUP_ID, 
                 BoardId);
        messenger.sendToBoard("server", reg);
        Serial.println("Registered with server.");
    }

    // 2. THE SAFETY GATE
    // If safety is NOT enabled, we stop updating the servo. 
    if (!safetyEnabled) {
        // Move to a specific "Safe" position.
        myServo.write(90); 
        
        // Optional: detach() if you want the servo to have NO holding torque (let it go limp)
        // if (myServo.attached()) myServo.detach();
        
        return; // Exit loop early so no sweep code runs
    }

    // Re-attach if we were detached and safety is now OK
    if (!myServo.attached()) {
        myServo.attach(SERVO_PIN);
    }

    // 3. CONTINUOUS MOVEMENT: Sweep the servo back and forth
    // We only reach this code if safetyEnabled == true
    if (millis() - lastMoveMs > 15) { // Slow the sweep down
        lastMoveMs = millis();
        
        servoPos += sweepDirection;
        if (servoPos >= 180 || servoPos <= 0) {
            sweepDirection *= -1; // Reverse direction
        }
        
        myServo.write(servoPos);
    }
}
