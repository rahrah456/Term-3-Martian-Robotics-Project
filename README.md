# 🚀 UCL Year 1 Robotics Challenge

# 🛸 Terraformer Rover (Haunter)

##### 👨‍🚀 Jonah Wilson-Troy, Wyatt Ip, Ibrahim Syawwal Sofian, Rahul Ranjan

An autonomous robot that navigates a 2.5 m x 2.5 m (actually 2.4 m x 2.4 m...) Martian-themed arena, plants seeds in marked holes, and returns to base — all autonomously and in collaboration with 13 other ~~obstacles~~ 🤖 rovers.

---

## 🚦 Getting Started

### 📋 Prerequisites

- 🧠 Arduino Giga R1 WiFi
- 🛠️ Arduino IDE (v2)
- 🐍 Python 3.9+ and `pip`

### ⚙️ Installation

- 📂 Open `final_code/final_code.ino` in the Arduino IDE.

- 🔑 Fill in `final_code/secrets.h` with your credentials:
  
  `WIFI_SSID` / `WIFI_PASSWORD` — your 2.4 GHz WiFi network
  
  `BROKER_HOST` — IP of your MQTT broker
  
  `GROUP_ID` — your team number

- 📚 Install the required libraries via the Library Manager:
  
  - ⚡ **Motoron** (for motor control)
  - 📡 **MFRC522_I2C** (RFID reader)
  - 📨 **MiniMessenger** (MQTT wrapper)
  - 🧭 **LIS3MDL** (Magnetometer library)
  - 📐 **LSM6** (Accelerometer library)
  - 🦾 **Servo** (built-in)
  - 🔌 **Wire** (built-in)
  - 📟 **Arduino** (built-in)
  - ➗ **Math** (built-in)

- ✅ Select your board and port, then upload.

### 📊 Dashboard

We provide a live dashboard 📈 served over HTTP that connects to your MQTT broker:

```bash
cd final_code
python3 -m venv venv
source venv/bin/activate       # Windows: venv\Scripts\activate
pip install paho-mqtt
python dashboard.py
```

🌐 Open `http://localhost:8081` — the dashboard shows a map view 🗺️, sensor readings 📡, PID tuning sliders 🎛️, test command buttons 🔘, and more.

---

## 📁 Repository Structure

```
final_code/              # Main robot sketch and dashboard
├── final_code.ino       # Entry point — setup + 50 Hz main loop + state machine
├── Config.h             # Pin assignments, motor limits, calibration constants
├── Map.h                # Arena map (9×9 hole grid, dynamic coordinate system)
├── Sensors.h            # IR array, UDSF, RDID, IMU, LDR
├── Control.h            # Motor control, non-blocking MotionSM, seed dispense
├── Localisation.h       # Dead-reckoning + RFID correction
├── MQTTManager.h        # MiniMessenger wrapper for MQTT + heartbeat watchdog
├── secrets.h            # WiFi & MQTT credentials (not tracked)
└── dashboard.py         # Python MQTT subscriber + HTTP/SSE server
archive/                 # Previous prototypes and test sketches
```

The robot code is split into **header files by concern**, all `#include`d by the single `.ino`. There is no `.cpp` — everything is in headers for simplicity.

---

## 🧠 How It Works

### 👁️ Sensing

- 🔦 **IR reflectance array** (9 sensors under the robot) detects line position via a centroid calculation. The IR emitters used every other cycle to account for ambient light.
- 📡 **Ultrasonic rangefinders** (left, mid, right) provide obstacle distance with median-of-3 noise rejection per sensor per cycle plus EMA smoothing (α=0.2).
- 🏷️ **RFID** (MFRC522 on I²C) reads base and hole tags. The tag data is parsed via `arena.rfidToHole()` to give grid coordinates.
- 🧭 **IMU** (LIS3MDL magnetometer + LSM6 accelerometer/gyro) gives a tilt-compensated heading, used as the absolute orientation reference.

### 📍 Localisation

Position is dead-reckoned from encoder deltas projected along the IMU heading. A small fraction of the accelerometer double-integration is blended in to help during wheel slip. When an RFID hole tag is read, the pose snaps to the known hole centre.

### 🏃 Motion

All motion is **non-blocking**. `MotionSM` is a struct with a `tick()` method that advances one step per call. Supported motion types:
- `STRAIGHT` / `TURN` — open-loop, encoder-counted
- `LINE_FOLLOW` — PID on IR centroid (with hole-crossing hold)
- `WALL_FOLLOW` — proportional control on side UDS (configurable side + target distance)
- `CENTRE_TUNNEL` — proportional on L−R UDS difference
- `AVOID_OBSTACLE` — multi-phase sensor-driven detour (see flowchart below)

The main loop calls `tick()` every 20 ms, passing current sensor data. This keeps MQTT, e-stop, and sensor reads responsive during movement.

Additionally, `ReviveMove` is a separate helper for open-loop forward with linear deceleration (used for REVIVE command).

### 🔄 State Machine

The main loop runs a simple state machine:

1. **INIT** — wait for IMU calibration, MQTT connection
2. **IDLE** — listen for ENABLE, transition to PLAN after 1 s cooldown
4. **PLAN** — choose closest unplanted hole via `arena.holeCentre()`
5. **NAVIGATE** — drive toward target: line-follow via PID on IR centroid when line visible, bearing-based steering when no line. Obstacle check on `filteredUdsM` each tick.
6. **AVOID** — on obstacle (`BLOCKED` or direct check in NAVIGATE), run the multi-phase `AVOID_OBSTACLE` detour (bidirectional, line-finding)
7. **DEPOSIT** — heading alignment → RFID search → server fertility query → position → dispense → mark planted
8. **RETURN_BASE** — navigate back to origin


### 📨 Communication

MQTT over WiFi using the **MiniMessenger** library. The robot:
- 📤 **Publishes** (at 5 Hz): pose, state, sensor snapshot (16 fields incl. seed index), hole status
- 📥 **Subscribes**: ENABLE/DISABLE, heartbeat (server watchdog ~1 s timeout), emergency, PID tuning (`kp`, `ki`, `kd`, `md`), test commands (`FOLLOW_LINE`, `FOLLOW_WALL`, `OBSTACLE_AVOIDANCE`, `DEPOSIT`, `EXIT_BASE`, `REVIVE`, `GRID_NAV`, `GRID_NAV_NOLINES`), heading reset (`HEADING:0`), seed select (`SEED:1`–`SEED:5`)

---

## 🏛️ Software Overview Diagram

The rover firmware is a single Arduino sketch split across header files by concern. All headers are `#include`d into `final_code.ino`, which owns the 50 Hz main loop and the high-level state machine. The Python dashboard communicates over MQTT.

```mermaid
graph TB
    subgraph "Arduino Giga R1 WiFi"
        INO["final_code.ino<br/>───────────────<br/>setup() · 50 Hz loop()<br/>State machine · MQTT callbacks<br/>Encoder polling (quadrature table)"]

        CFG["Config.h<br/>───────────<br/>Pin assignments · Motor limits<br/>Robot dimensions · Calibration<br/>Tick-fit formulae<br/>(ticksForDistance, ticksForTurn)"]

        SEN["Sensors.h<br/>──────────<br/>IR array (readIR, irCentroid)<br/>UDSManager round-robin L→M→R<br/>median-of-3 + EMA filter<br/>IMU tilt-compensated heading<br/>Light sensor · Bumper (TODO)"]

        CTL["Control.h<br/>──────────<br/>setMotors() · Motor bias (1.047)<br/>MotionSM: STRAIGHT, TURN,<br/>LINE_FOLLOW, WALL_FOLLOW,<br/>CENTRE_TUNNEL, AVOID_OBSTACLE<br/>ReviveMove (open-loop decel)<br/>Seed dispensing (servo, 5 seeds)"]

        LOC["Localisation.h<br/>──────────────<br/>Dead-reckoning (encoders + heading)<br/>Heading filter: gyro integration +<br/>mag correction (motors on) /<br/>sin-cos vector EMA (motors off)<br/>Slip detection · RFID snap correction"]

        MAP["Map.h<br/>──────<br/>9×9 grid geometry (250 mm spacing)<br/>Dynamic coordinate origin<br/>Logical↔physical coordinate transform<br/>Hole centres · Arena boundaries"]

        MQT["MQTTManager.h<br/>─────────────<br/>MiniMessenger wrapper<br/>Heartbeat watchdog (1 s timeout)<br/>Pub: pose, state, sensor snapshot<br/>Sub: enable, PID, test cmds,<br/>heading reset, seed select"]

        SEC["secrets.h<br/>──────────<br/>WiFi SSID/password<br/>Broker host · Port · Group ID<br/>DASHBOARD_ID"]
    end

    subgraph "Python Dashboard"
        DSH["dashboard.py<br/>──────────────<br/>HTTP/SSE server (port 8081)<br/>Live map · Sensor telemetry<br/>PID tuning sliders · Test buttons<br/>Hole status editor<br/>Console for custom MQTT commands"]
    end

    subgraph "External"
        BRK["MQTT Broker"]
        SRV["Arena Server<br/>(heartbeat, fertility,<br/>airlock, emergency)"]
    end

    INO --> CFG
    INO --> SEN
    INO --> CTL
    INO --> LOC
    INO --> MAP
    INO --> MQT
    MQT --> SEC

    MQT <-->|"WiFi 2.4 GHz"| BRK
    DSH <-->|"paho-mqtt"| BRK
    BRK <--> SRV

    subgraph "Enable Chain"
        direction LR
        K["!killed<br/>(hw button)"] --> S["serverAllow<br/>(heartbeat)"]
        S --> D["dashboardDesired<br/>(MQTT ENABLE)"]
        D --> E(["Motors allowed"])
    end
```

---

## 📊 Behaviour Flowcharts

### 🎯 Main State Machine

The top-level state machine runs inside `loop()` at 50 Hz. State transitions happen only when `MotionSM` is `IDLE` (no active motion in progress). The enable chain (`!killed && serverAllow && dashboardDesired`) gates all motor activity.

```mermaid
stateDiagram-v2
    [*] --> ST_INIT

    ST_INIT --> ST_IDLE : IMU calibrated,<br/>MQTT connected

    ST_IDLE --> ST_PLAN : Localisation valid<br/>& 1 s cooldown

    ST_PLAN --> ST_NAVIGATE : Target hole selected<br/>(closest unplanted)
    ST_PLAN --> ST_RETURN_BASE : All holes planted

    ST_NAVIGATE --> ST_DEPOSIT : Within 40% of<br/>hole spacing distance
    ST_NAVIGATE --> ST_AVOID : Obstacle detected<br/>(mid UDS < threshold<br/>or motion BLOCKED)

    ST_AVOID --> ST_PLAN : Avoidance complete<br/>(MotionSM → IDLE)

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
        • Line visible → PID on IR centroid
        • No line → bearing-based steering using IMU
        Obstacle check using filtered UDS data every tick.
    end note
```

### ⏱️ 50 Hz Main Loop

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

### 🚧 Obstacle Avoidance Sequence

When an obstacle is detected (either via `BLOCKED` return from `STRAIGHT`/`TURN`, or directly in `runNavigate()` via `filteredUdsM`), the main loop transitions to `ST_AVOID`. `runAvoid()` starts the sensor-driven `AVOID_OBSTACLE` sequence inside `MotionSM`, which advances through four phases autonomously.

```mermaid
flowchart TB
    DETECT(["Obstacle detected<br/>(mid UDS < threshold<br/>or motion BLOCKED)"]) --> DIR

    DIR{"avoidDirection = 0?"}
    DIR -->|Yes| PICK["Pick direction by clearance:<br/>diff = udsLeft − udsRight<br/>diff > 15 cm → CW (hug left)<br/>else → CCW (hug right)"]
    DIR -->|No, already set| P0

    PICK --> P0

    subgraph "Phase 0 — Spin Until Clear"
        P0["Spin in chosen direction<br/>(avoidDirection × speed)"]
        P0 --> P0_CHK{"Mid UDS > 25 cm<br/>OR 2 s timeout?"}
        P0_CHK -->|No| P0
        P0_CHK -->|Yes| P1
    end

    subgraph "Phase 1 — Hug Side"
        P1["Drive forward, hug<br/>chosen side with PID"]
        P1_CTRL["sideDist =<br/>avoidDirection>0 → udsLeft<br/>else → udsRight<br/>Error = sideDist − 15 cm<br/>Correction = error × 12.0<br/>Left = base + dir × corr<br/>Right = base − dir × corr"]
        P1 --> P1_CTRL
        P1_CTRL --> P1_CHK{"sideDist > 35 cm?<br/>(cleared obstacle)"}
        P1_CHK -->|No| P1
        P1_CHK -->|Yes| P2
    end

    subgraph "Phase 2 — Clear + Swing Back"
        P2["Drive straight<br/>(500 ms)"]
        P2 --> P2_SWING["Swing back (opposite<br/>to initial turn, 600 ms)"]
        P2_SWING --> P3
    end

    subgraph "Phase 3 — Find & Centre on Line"
        P3["Crawl forward (300 mm/s)"]
        P3 --> P3_CHK{"Centroid<br/>detected?"}
        P3_CHK -->|No, < 5 s| P3
        P3_CHK -->|Timeout 5 s| DONE_TIMEOUT(["DONE (no line found)"])
        P3_CHK -->|Yes| P3_CTRL["Proportional centering:<br/>error = centroid − 4000<br/>correction = error × 0.15<br/>cap ±60<br/>300 mm/s + steer"]
        P3_CTRL --> P3_OK{"|error| < 200?"}
        P3_OK -->|No| P3_CTRL
        P3_OK -->|Yes| DONE(["DONE → ST_PLAN"])
    end
```

### 🌱 Deposit Sequence

The deposit sequence is a **blocking routine** with its own internal sensor-and-MQTT polling loop. It aligns the rover, searches for the RFID tag at the hole, queries the server for fertility, positions the seed chute, dispenses a seed, and marks the hole planted.

```mermaid
flowchart TB
    START(["runDeposit() → blocking"]) --> ALIGN

    ALIGN["1. Align to nearest cardinal heading<br/>(0°, 90°, 180°, 270°)<br/>via motion.startTurn() + blocking tick"]
    ALIGN --> SEARCH

    SEARCH["2. Drive forward (heading hold +<br/>IR line assist) while scanning<br/>for RFID tag — up to 15 s"]
    SEARCH --> TAG{"Tag<br/>found?"}
    TAG -->|No| ABORT1(["Abort: no tag found"])
    TAG -->|Yes| QUERY

    QUERY["3. sendIsFertile(rfidBuf)<br/>Wait up to 10 s for server reply<br/>(re-send every 2 s)"]
    QUERY --> REPLY{"Server<br/>replied?"}
    REPLY -->|No| ABORT2(["Abort: no server reply"])
    REPLY -->|Yes| FERTILE{"Hole<br/>fertile?"}
    FERTILE -->|No| ABORT3(["Abort: hole not fertile"])
    FERTILE -->|Yes| POSITION

    POSITION["4. motion.startStraight()<br/>DEPOSIT_EXTRA_MM<br/>(aligns chute over hole)"]
    POSITION --> DISPENSE

    DISPENSE["5. dispenseNextSeed(servo)<br/>Servo advances to next slot<br/>(g_seedIdx auto-increments)"]
    DISPENSE --> WIGGLE

    WIGGLE["6. Wiggle fwd/back<br/>(30 mm each) to clear chute<br/>(two startStraight calls)"]
    WIGGLE --> LOCK["lockSeeds(servo)<br/>Servo → angle 0 (locked)"]
    LOCK --> MARK

    MARK["7. holePlanted = true<br/>sendSeedPlanted() via MQTT<br/>sendHoleStatus() to dashboard"]
    MARK --> DONE(["Return → ST_PLAN"])
```

### 🏠 Base Exit Sequence

The base exit navigates a fixed 6-leg path through the base, scans the exit RFID, requests airlock permission from the server (re-sending every 2 s), traverses the tunnel using UDS-based centring, detects doors via front UDS, and detects the end of the tunnel using IMU pitch change (gravity normalisation).

```mermaid
flowchart TB
    START(["runBaseExit() → blocking"]) --> L1

    L1["Leg 1: Forward 358 mm<br/>(clear base walls)"]
    L1 --> T1["Turn right 90°"]
    T1 --> L2["Leg 2: Forward 330 mm"]
    L2 --> T2["Turn left 90°"]
    T2 --> L3["Leg 3: Forward 178 mm<br/>(align RFID with exit tag)"]
    L3 --> RFID

    RFID["RFID scan: 5 forward steps<br/>(20 mm each)<br/>then 5 backward steps<br/>(20 mm each)"]
    RFID --> RFID_CHK{"Tag<br/>found?"}
    RFID_CHK -->|No| ABORT(["Abort: no RFID tag"])
    RFID_CHK -->|Yes| AIRLOCK

    AIRLOCK["sendAirlockRequest('B', tag)<br/>Wait up to 15 s<br/>Re-send every 2 s"]
    AIRLOCK --> AIR_CHK{"airlockAccepted<br/>= true?"}
    AIR_CHK -->|No| DENY(["Abort: airlock denied"])
    AIR_CHK -->|Yes| L4

    L4["Leg 4: Forward 232 mm"]
    L4 --> T3["Turn left 90°"]
    T3 --> L5["Leg 5: Forward 330 mm"]
    L5 --> T4["Turn right 90°"]
    T4 --> L6["Leg 6: Forward 252 mm<br/>(to tunnel entrance)"]
    L6 --> DOOR_WAIT

    DOOR_WAIT["Wait for first tunnel door:<br/>If already open → proceed<br/>If closed (UDS < 30 cm)<br/>→ wait for > 50 cm<br/>(10 s timeout each)"]
    DOOR_WAIT --> TUNNEL

    TUNNEL["Enter tunnel:<br/>motion.startTunnelCentre()<br/>Proportional on L−R UDS<br/>(kp 1.0, speed 660)"]
    TUNNEL --> DOOR2{"Front UDS<br/>< 30 cm?"}
    DOOR2 -->|Yes| WAIT2["Stop, wait for 2nd door<br/>to open (> 50 cm, 15 s timeout)"]
    DOOR2 -->|No, tunnel continues| TUNNEL
    WAIT2 --> EXIT_DRIVE

    EXIT_DRIVE["Drive forward at tunnel speed<br/>while scanning IMU pitch"]

    EXIT_DRIVE --> PITCH{"Pitch diff > 8°<br/>then back < 3°?"}
    PITCH -->|No| EXIT_DRIVE
    PITCH -->|Yes| STOP["setMotors(0, 0)"]
    STOP --> DONE(["Base exit complete → ST_IDLE"])
```

---

### 💪 Revive Sequence

The REVIVE command executes an open-loop forward push with linear deceleration, using the dedicated `ReviveMove` helper. It pushes another robot 🦾 out of the way with controlled momentum.

```mermaid
flowchart TB
    START(["REVIVE command"]) --> INIT["ReviveMove::start(totalTicks)<br/>distance = 2 × holeSpacing − chassisLen"]

    INIT --> RUN["ReviveMove::tick()<br/>each 20 ms cycle"]
    RUN --> DECEL{"Speed > 0?"}
    DECEL -->|Yes| PUSH["setMotors(speed, speed)<br/>Linear deceleration ramp"]
    PUSH --> RUN
    DECEL -->|No| STOP(["setMotors(0, 0)"])
    STOP --> DONE(["DONE → ST_IDLE"])
```

### 🗺️ Grid Navigation Sequence

The grid navigation commands (`GRID_NAV`, `GRID_NAV_NOLINES`) drive a fixed 5-node L-shaped path, attempting to detect RFID tags at each node to snap the pose.

```mermaid
flowchart TB
    START(["runGridNav() / runGridNavNoLines()"]) --> LEG1

    LEG1["Leg 1: Forward 250 mm<br/>(1 node spacing)<br/>driveSegment() with heading hold"]
    LEG1 --> NODE1{"RFID<br/>found?"}
    NODE1 -->|Yes| SNAP1["Server query → correctFromRFID()<br/>Snap pose to hole centre"]
    NODE1 -->|No| CONT1
    SNAP1 --> CONT1
    CONT1["If not found, backtrack 50 mm<br/>and try ±5° (3 attempts)"]

    CONT1 --> LEG1B["Leg 1b: Forward 250 mm<br/>(2nd node)"]
    LEG1B --> NODE2
    NODE2 --> SNAP2
    SNAP2 --> TURN1

    TURN1["Turn right 90°<br/>(motion.startTurn)"]
    TURN1 --> LEG2["Leg 2: Forward 250 mm<br/>(3rd node)<br/>try ±5° if no RFID"]

    LEG2 --> TURN2["Turn left 90°"]

    TURN2 --> LEG3["Leg 3: Forward 250 mm<br/>(4th node)"]
    LEG3 --> LEG3B["Leg 3b: Forward 250 mm<br/>(5th node)"]
    LEG3B --> DONE(["DONE → ST_IDLE"])
```

---

## 🎛️ PID System Overview

Line-following and wall-following use PID controllers whose gains are tuneable at runtime via MQTT. The dashboard has dedicated sliders for `kp`, `ki`, `kd`, and `maxDiff`.

```mermaid
flowchart LR
    subgraph "Sensors"
        IR["IR centroid<br/>(0–8000, 4000 = centre)"]
        UDS_L["UDS left (cm)"]
        UDS_R["UDS right (cm)"]
    end

    subgraph "MQTT Tuning"
        KP["kp slider<br/>ki slider<br/>kd slider<br/>md slider"]
    end

    subgraph "MotionSM tick()"
        ERR["error = setpoint − sensor<br/>e.g. centroid − 4000<br/>or sideDist − targetDist"]
        PID["PID calculation:<br/>P = kp × error<br/>I = ki × ∫ error dt<br/>D = kd × d(error)/dt<br/>correction = P + I + D"]
        CLAMP["constrain(correction,<br/>−maxDiff, +maxDiff)"]
        MOTOR["left = base + correction<br/>right = base − correction<br/>constrain to [300, 660]"]
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
