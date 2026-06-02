# UCL Year 1 Robotics Challenge

# Terraformer Rover (Haunter)

##### Jonah Wilson-Troy, Wyatt Ip, Ibrahim Syawwal Sofian, Rahul Ranjan

An autonomous robot that navigates a 2.5 m x 2.5 m (actually 2.4 m x 2.4 m...) Martian-themed arena, plants seeds in marked holes, and returns to base -- all autonomously and in collaboration with 13 other rovers.

---

## Getting Started

### Prerequisites

- Arduino Giga R1 WiFi
- Arduino IDE (v2)
- Python 3.9+ and `pip`

### Installation

- Open `final_code/final_code.ino` in the Arduino IDE.

- Fill in `final_code/secrets.h` with your credentials:
  
  `WIFI_SSID` / `WIFI_PASSWORD` -- your 2.4 GHz WiFi network
  
  `BROKER_HOST` -- IP of your MQTT broker
  
  `GROUP_ID` -- your team number

- Install the required libraries via the Library Manager:
  
  - **Motoron** (for motor control)
  - **MFRC522_I2C** (RFID reader)
  - **MiniMessenger** (MQTT wrapper)
  - **LIS3MDL** (Magnetometer library)
  - **LSM6** (Accelerometer library)
  - **Servo** (built-in)
  - **Wire** (built-in)
  - **Arduino** (built-in)
  - **Math** (built-in)

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

Open `http://localhost:8081` -- the dashboard shows a map view, sensor readings, PID tuning sliders, test command buttons, and more.

---

## Repository Structure

```
final_code/              # Main robot sketch and dashboard
├── final_code.ino       # Entry point -- setup + 50 Hz main loop + state machine
├── Config.h             # Pin assignments, motor limits, calibration constants
├── Map.h                # Arena map (9x9 hole grid, dynamic coordinate system)
├── Sensors.h            # IR array, UDSF, RDID, IMU, LDR
├── Control.h            # Motor control, non-blocking MotionSM, seed dispense
├── Localisation.h       # Dead-reckoning + RFID correction
├── MQTTManager.h        # MiniMessenger wrapper for MQTT + heartbeat watchdog
├── secrets.h            # WiFi & MQTT credentials (not tracked)
└── dashboard.py         # Python MQTT subscriber + HTTP/SSE server
archive/                 # Previous prototypes and test sketches
```

The robot code is split into **header files by concern**, all `#include`d by the single `.ino`. There is no `.cpp` -- everything is in headers for simplicity.

---

## How It Works

### Sensing

- **IR reflectance array** (9 sensors under the robot) detects line position via a centroid calculation. The IR emitters are used every other cycle to account for ambient light.
- **Ultrasonic rangefinders** (left, mid, right) provide obstacle distance with median-of-3 noise rejection per sensor per cycle plus EMA smoothing (alpha=0.2).
- **RFID** (MFRC522 on I2C) reads base and hole tags. The tag data is parsed via `arena.rfidToHole()` to give grid coordinates.
- **IMU** (LIS3MDL magnetometer + LSM6 accelerometer/gyro) gives a tilt-compensated heading, used as the absolute orientation reference.

### Localisation

Position is dead-reckoned from encoder deltas projected along the IMU heading. A small fraction of the accelerometer double-integration is blended in to help during wheel slip. When an RFID hole tag is read, the pose snaps to the known hole centre.

### Motion

All motion is **non-blocking**. `MotionSM` is a struct with a `tick()` method that advances one step per call. Supported motion types:

- `STRAIGHT` / `TURN` -- open-loop, encoder-counted
- `LINE_FOLLOW` -- PID on IR centroid (with hole-crossing hold)
- `WALL_FOLLOW` -- proportional control on side UDS (configurable side + target distance)
- `CENTRE_TUNNEL` -- proportional on L-R UDS difference
- `AVOID_OBSTACLE` -- multi-phase sensor-driven detour (see flowchart below)

The main loop calls `tick()` every 20 ms, passing current sensor data. This keeps MQTT, e-stop, and sensor reads responsive during movement.

Additionally, `ReviveMove` is a separate helper for open-loop forward with linear deceleration (used for REVIVE command).

### State Machine

The main loop runs a simple state machine:

1. **INIT** -- wait for IMU calibration, MQTT connection
2. **IDLE** -- listen for ENABLE, transition to PLAN after 1 s cooldown
3. **PLAN** -- choose closest unplanted hole via `arena.holeCentre()`
4. **NAVIGATE** -- drive toward target: line-follow via PID on IR centroid when line visible, bearing-based steering when no line. Obstacle check on `filteredUdsM` each tick.
5. **AVOID** -- on obstacle (`BLOCKED` or direct check in NAVIGATE), run the multi-phase `AVOID_OBSTACLE` detour (bidirectional, line-finding)
6. **DEPOSIT** -- heading alignment, RFID search, server fertility query, position, dispense, mark planted
7. **RETURN_BASE** -- navigate back to origin

### Communication

MQTT over WiFi using the **MiniMessenger** library. The robot:

- **Publishes** (at 5 Hz): pose, state, sensor snapshot (16 fields incl. seed index), hole status
- **Subscribes**: ENABLE/DISABLE, heartbeat (server watchdog ~1 s timeout), emergency, PID tuning (`kp`, `ki`, `kd`, `md`), test commands (`FOLLOW_LINE`, `FOLLOW_WALL`, `OBSTACLE_AVOIDANCE`, `DEPOSIT`, `EXIT_BASE`, `REVIVE`, `GRID_NAV`, `GRID_NAV_NOLINES`), heading reset (`HEADING:0`), seed select (`SEED:1`-`SEED:5`)

---

## Software Overview Diagram

The rover firmware is a single Arduino sketch split across header files by concern. All headers are `#include`d into `final_code.ino`, which owns the 50 Hz main loop and the high-level state machine. The Python dashboard communicates over MQTT.

<img width="1600" height="710" alt="WhatsApp Image 2026-05-31 at 16 12 47" src="https://github.com/user-attachments/assets/39dcf725-35b6-480d-bc9d-415b2547e722" />
<img width="281" height="601" alt="WhatsApp Image 2026-05-31 at 16 12 48" src="https://github.com/user-attachments/assets/64862510-18dc-49c4-9389-aa47b500512e" />

---

## Behaviour Flowcharts

### Main State Machine

The top-level state machine runs inside `loop()` at 50 Hz. State transitions happen only when `MotionSM` is `IDLE` (no active motion in progress). The enable chain (`!killed && serverAllow && dashboardDesired`) gates all motor activity.

```mermaid
stateDiagram-v2
    [*] --> ST_INIT

    ST_INIT --> ST_IDLE : IMU calibrated,<br/>MQTT connected

    ST_IDLE --> ST_PLAN : Localisation valid<br/>& 1 s cooldown

    ST_PLAN --> ST_NAVIGATE : Target hole selected<br/>(closest unplanted)
    ST_PLAN --> ST_RETURN_BASE : All holes planted

    ST_NAVIGATE --> ST_DEPOSIT : Within 40% of<br/>hole spacing distance
    ST_NAVIGATE --> ST_AVOID : Obstacle detected<br/>(mid UDS < threshold<br/>or motion BLOCKED)

    ST_AVOID --> ST_PLAN : Avoidance complete<br/>(MotionSM -> IDLE)

    ST_DEPOSIT --> ST_PLAN : Seed dispensed<br/>& hole marked planted

    ST_RETURN_BASE --> ST_IDLE : Arrived at base origin

    ST_EXIT_BASE --> ST_IDLE : Tunnel traversed,<br/>gravity normalised

    ST_REVIVE --> ST_IDLE : Push complete

    note right of ST_IDLE
        Motors stopped.
        Enable chain gating:
        !killed && serverAllow
        && dashboardDesired.
        Waiting for ENABLE.
    end note

    note right of ST_NAVIGATE
        Dual-mode steering:
        . Line visible -> PID on IR centroid
        . No line -> bearing-based steering using IMU
        Obstacle check using filtered UDS data every tick.
    end note
```

### 50 Hz Main Loop

Every 20 ms, the loop reads all sensors, publishes telemetry, checks the safety chain, ticks the motion controller, and then runs the current state behaviour.

```mermaid
graph LR
    START(["loop()"]) --> MQTT["mqtt.loop()"]
    MQTT --> SERIAL{"Serial cmd?"}
    SERIAL -->|enable/dsbl/kill| SER_CMD["apply serial cmd"]
    SERIAL -->|no| ESTOP
    SER_CMD --> ESTOP
    ESTOP["handleEStop()"]
    ESTOP --> SYNC{"20ms tick?"}
    SYNC -->|no| PUB
    SYNC -->|yes| SENSE["readIR, encoders, uds.tick, readIMU, loc.update"]
    SENSE --> PUB{"200ms pub?"}
    PUB -->|yes| MSEND["sendPose/State/Sensors"]
    PUB -->|no| KILL
    MSEND --> KILL
    KILL{"killed || not enabled?"}
    KILL -->|yes| HALT["setMotors(0)"]
    KILL -->|no| MOT{"MotionSM active?"}
    MOT -->|idle| SW_LOOP{"state"}
    MOT -->|running| TICK["motion.tick(...)"]
    TICK --> BLK{"BLOCKED?"}
    BLK -->|yes| STAV["state=ST_AVOID"]
    BLK -->|no| DONE(["done"])
    STAV --> DONE

    subgraph "State Dispatch"
        SW_LOOP -->|INIT| RI["runInit()"]
        SW_LOOP -->|IDLE| RID["runIdle()"]
        SW_LOOP -->|PLAN| RP["runPlan()"]
        SW_LOOP -->|NAV| RN["runNavigate()"]
        SW_LOOP -->|AVOID| RA["runAvoid()"]
        SW_LOOP -->|DEP| RD["runDeposit()"]
        SW_LOOP -->|RTN| RR["runReturnBase()"]
        SW_LOOP -->|EXIT| RE["runBaseExit()"]
        SW_LOOP -->|REVIVE| RV["runRevive()"]
        SW_LOOP -->|TEST| SK["(callback)"]
        RI --> DONE; RID --> DONE; RP --> DONE; RN --> DONE
        RA --> DONE; RD --> DONE; RR --> DONE; RE --> DONE
        RV --> DONE; SK --> DONE
    end
```

### Obstacle Avoidance Sequence

When an obstacle is detected (either via `BLOCKED` return from `STRAIGHT`/`TURN`, or directly in `runNavigate()` via `filteredUdsM`), the main loop transitions to `ST_AVOID`. `runAvoid()` starts the sensor-driven `AVOID_OBSTACLE` sequence inside `MotionSM`, which advances through four phases autonomously.

<img width="169" height="721" alt="WhatsApp Image 2026-05-31 at 16 12 48 (1)" src="https://github.com/user-attachments/assets/7d6cae25-3ec4-4580-a27c-098e6a0102d0" />


### Deposit Sequence

The deposit sequence is a **blocking routine** with its own internal sensor-and-MQTT polling loop. It aligns the rover, searches for the RFID tag at the hole, queries the server for fertility, positions the seed chute, dispenses a seed, and marks the hole planted.

<img width="169" height="961" alt="WhatsApp Image 2026-05-31 at 16 12 48 (2)" src="https://github.com/user-attachments/assets/0304cdfb-ee0c-4d88-a151-c6c8290ec305" />

### Base Entrance Sequence

<img width="169" height="1561" alt="WhatsApp Image 2026-05-31 at 16 12 47 (1)" src="https://github.com/user-attachments/assets/ec89b855-b331-412d-838c-2f81001e30ff" />


### Base Exit Sequence

The base exit navigates a fixed 6-leg path through the base, scans the exit RFID, requests airlock permission from the server (re-sending every 2 s), traverses the tunnel using UDS-based centring, detects doors via front UDS, and detects the end of the tunnel using IMU pitch change (gravity normalisation).

<img width="169" height="841" alt="WhatsApp Image 2026-05-31 at 16 12 47 (2)" src="https://github.com/user-attachments/assets/f9fa0a4b-e8a8-46bf-8f3a-d4cd2a62972a" />


---

### Revive Sequence

The REVIVE command executes an open-loop forward push with linear deceleration, using the dedicated `ReviveMove` helper. It pushes another robot out of the way with controlled momentum.

```mermaid
flowchart TB
    START(["REVIVE command"]) --> INIT["ReviveMove::start(totalTicks)<br/>distance = 2 x holeSpacing - chassisLen"]

    INIT --> RUN["ReviveMove::tick()<br/>each 20 ms cycle"]
    RUN --> DECEL{"Speed > 0?"}
    DECEL -->|Yes| PUSH["setMotors(speed, speed)<br/>Linear deceleration ramp"]
    PUSH --> RUN
    DECEL -->|No| STOP(["setMotors(0, 0)"])
    STOP --> DONE(["DONE -> ST_IDLE"])
```

### Grid Navigation Sequence

The grid navigation commands (`GRID_NAV`, `GRID_NAV_NOLINES`) drive a fixed 5-node L-shaped path, attempting to detect RFID tags at each node to snap the pose.

```mermaid
flowchart TB
    START(["runGridNav() / runGridNavNoLines()"]) --> LEG1

    LEG1["Leg 1: Forward 250 mm<br/>(1 node spacing)<br/>driveSegment() with heading hold"]
    LEG1 --> NODE1{"RFID<br/>found?"}
    NODE1 -->|Yes| SNAP1["Server query -> correctFromRFID()<br/>Snap pose to hole centre"]
    NODE1 -->|No| CONT1
    SNAP1 --> CONT1
    CONT1["If not found, backtrack 50 mm<br/>and try +/-5 deg (3 attempts)"]

    CONT1 --> LEG1B["Leg 1b: Forward 250 mm<br/>(2nd node)"]
    LEG1B --> NODE2
    NODE2 --> SNAP2
    SNAP2 --> TURN1

    TURN1["Turn right 90 deg<br/>(motion.startTurn)"]
    TURN1 --> LEG2["Leg 2: Forward 250 mm<br/>(3rd node)<br/>try +/-5 deg if no RFID"]

    LEG2 --> TURN2["Turn left 90 deg"]

    TURN2 --> LEG3["Leg 3: Forward 250 mm<br/>(4th node)"]
    LEG3 --> LEG3B["Leg 3b: Forward 250 mm<br/>(5th node)"]
    LEG3B --> DONE(["DONE -> ST_IDLE"])
```

---

## PID System Overview

Line-following and wall-following use PID controllers whose gains are tuneable at runtime via MQTT. The dashboard has dedicated sliders for `kp`, `ki`, `kd`, and `maxDiff`.

```mermaid
flowchart LR
    subgraph "Sensors"
        IR["IR centroid<br/>(0-8000, 4000 = centre)"]
        UDS_L["UDS left (cm)"]
        UDS_R["UDS right (cm)"]
    end

    subgraph "MQTT Tuning"
        KP["kp slider<br/>ki slider<br/>kd slider<br/>md slider"]
    end

    subgraph "MotionSM tick()"
        ERR["error = setpoint - sensor<br/>e.g. centroid - 4000<br/>or sideDist - targetDist"]
        PID["PID calculation:<br/>P = kp x error<br/>I = ki x integral error dt<br/>D = kd x d(error)/dt<br/>correction = P + I + D"]
        CLAMP["constrain(correction,<br/>-maxDiff, +maxDiff)"]
        MOTOR["left = base + correction<br/>right = base - correction<br/>constrain to [300, 660]"]
    end

    IR --> ERR
    UDS_L --> ERR
    UDS_R --> ERR
    KP --> PID
    ERR --> PID
    PID --> CLAMP
    CLAMP --> MOTOR
    MOTOR --> OUT["setMotors(left, right)"]
```

---

## Calibration

The robot requires calibration of its encoder-based motion to achieve accurate dead-reckoning.



### Motor power bias



### Move Calibration

This plots measured distance against encoder ticks for forward movement trials, showing the linear and exponential components of the fitted curve.

![Move calibration](README_resources/move_calib.png)

### Turn Calibration

This plots measured rotation angle against encoder ticks for turning trials, showing the linear fit used by `ticksForTurn()`.

![Turn calibration](README_resources/turn_calib.png)

#### Methodology

(To be filled in)
