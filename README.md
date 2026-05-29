# UCL Year 1 Robotics Challenge

# Terraformer Rover

##### Jonah Wilson-Troy, Wyatt Ip, Ibrahim Syawwal Sofian, Rahul Ranjan

An autonomous robot that navigates a 2.5 m x 2.5 m (actually 2.4 m x 2.4 m...) Martian-themed arena, plants seeds in marked holes, and returns to base - all autonomously and in collaboration with 13 other ~~obstacles~~ rovers.

---

## Getting Started

### Prerequisites

- Arduino Giga R1 WiFi
- Arduino IDE (v2)
- Python 3.9+ and `pip`

### Installation

- Open `final_code/final_code.ino` in the Arduino IDE.

- Fill in `final_code/secrets.h` with your credentials:
  
  `WIFI_SSID` / `WIFI_PASSWORD` - your 2.4 GHz WiFi network
  
  `BROKER_HOST` - IP of your MQTT broker
  
  `GROUP_ID` - your team number

- Install the required libraries via the Library Manager:
  
  - **Motoron** (for motor control)
  - **MFRC522_I2C** (RFID reader)
  - **MiniMessenger** (MQTT wrapper)
  - **Servo** (built-in)
  - **Wire** (built-in)
  - **Arduino** (built-in)

- Select your board and port, then upload.

### Dashboard

We provide a live dashboard served over HTTP that connects to your MQTT broker:

```bash
cd final_code
python3 -m venv venv
source venv/bin/activate       # Windows: venv\Scripts\activate
pip install paho-mqtt
python dashboard.py
```

Open the URL `http://localhost:8081`. The dashboard shows a map view, sensor readings, PID tuning sliders, and test command buttons, and more.

---

## Repository Structure

```
final_code/              # Main robot sketch and dashboard
├── final_code.ino       # Entry point - setup + 50 Hz main loop
├── Config.h             # Pin assignments, motor limits, calibration
├── Map.h                # Arena layout (9×9 hole grid, walls, tunnels)
├── Sensors.h            # IR array, UDS, RFID, IMU drivers
├── Control.h            # PIDSpeed + non-blocking MotionSM
├── Localisation.h       # Dead-reckoning + RFID correction
├── MQTTManager.h        # MiniMessenger wrapper for MQTT
├── secrets.h            # WiFi & MQTT credentials (not tracked)
└── dashboard.py         # Python MQTT subscriber + HTTP/SSE server
archive/                 # Previous prototypes and test sketches
```

The robot code is split into header files by concern, all `#include`d by the single `.ino`. There is no `.cpp` and everything is in headers for simplicity.

---

## How It Works

### Sensing

- **IR reflectance array** (9 sensors under the robot) detects line position via a centroid calculation. The IR emitters are charged and discharged each cycle to cancel ambient light.
- **Ultrasonic rangefinders** (left, mid, right) provide obstacle distance with a rolling median filter.
- **RFID** (MFRC522 on I²C) reads hole tags. The tag data is parsed via `arena.rfidToHole()` to give grid coordinates.
- **IMU** (LIS3MDL magnetometer + LSM6 accelerometer/gyro) gives a tilt-compensated heading, used as the absolute orientation reference.

### Localisation

Position is dead-reckoned from encoder deltas projected along the IMU heading. A small fraction of the accelerometer double-integration is blended in to help during wheel slip. When an RFID hole tag is read, the pose snaps to the known hole centre.

### Motion

All motion is non-blocking. `MotionSM` is a struct with a `tick()` method that advances one step per call (straight line, turn, line follow, wall follow, or centre-in-tunnel). The main loop calls `tick()` every 20 ms, passing current sensor data. This keeps MQTT, e‑stop, and sensor reads responsive during movement.

### State Machine

The main loop runs a simple state machine:

1. **INIT** - wait for IMU calibration and MQTT connection
2. **IDLE** - listen for ENABLE command from dashboard
3. **PLAN** - choose next unplanted hole (closest first)
4. **NAVIGATE** - drive to target using straight-line segments and turns, line-following when crossing the grid
5. **AVOID** - on obstacle, execute a three-phase detour (turn, straight, turn) then resume navigation
6. **DEPOSIT** - at the hole, dispense seed, mark hole planted
7. **RETURN_BASE** - navigate back to origin after all seeds are planted

### Communication

MQTT over WiFi using the MiniMessenger library. The robot publishes pose, state, sensor snapshots, and hole status at 2 Hz. The dashboard can send ENABLE/DISABLE, PID tuning updates, hole status changes, and test commands (FOLLOW_LINE, FOLLOW_WALL, DEPOSIT).

---

## Software Overview Diagram

*Coming soon.*

---

## Behaviour Flowcharts

*Coming soon - we'll add flowcharts for the main state machine and the obstacle avoidance sequence.*
