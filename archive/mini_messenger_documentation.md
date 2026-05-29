# Robotics Challenge '26 MiniMessenger
[![Arduino Lint](https://github.com/Alexandros-Charitonidis/RoboticsChallenge-MQTT/actions/workflows/lint.yml/badge.svg)](https://github.com/Alexandros-Charitonidis/RoboticsChallenge-MQTT/actions/workflows/lint.yml)

This library handles the primary communication between the arena robots and the central challenge server.
Classroom board-to-board and board-to-server messaging for Arduino GIGA robot pairs.

## Description

The `MiniMessenger` library simplifies WiFi and MQTT communication for robotics projects. It provides a group-and-board based API, allowing students to easily send messages between specific boards or broadcast to an entire group without managing raw MQTT topics or connection retries.

### Key Features
- **Automatic Connection Management:** Handles WiFi and MQTT connection and reconnection logic.
- **Direct Messaging:** Send data or text to a specific board ID.
- **Group Broadcasting:** Broadcast messages to all boards in the same group.
- **Message Callbacks:** Simple event-driven architecture for handling incoming messages.
- **Lightweight:** Designed to run efficiently on the Arduino GIGA.

## Topic Structure

While the library abstracts topics away, they follow a standardized format compatible with tools like MQTT Explorer:

| Message Type | Topic Pattern |
|--------------|---------------|
| **Status** | `lab/g/{groupId}/board/{boardId}/status` |
| **Direct** | `lab/g/{groupId}/from/{fromId}/to/{targetId}` |
| **Group**  | `lab/g/{groupId}/from/{fromId}/to/all` |

- `{groupId}`: Your assigned team number (e.g., "1").
- `{boardId}`: Your unique board name within the group (e.g., "Terminator" or "KrabbyPatty").
- **Status Payloads:** "online" (with retain flag) or "offline" (via LWT).

## Installation

This library requires the **ArduinoMqttClient** library. You can install it via the Arduino Library Manager. If MiniMessenger is installed through the Arduino library manager, the external library should be installed automatically.

## Bare Minimum Code

To use the library, you need to provide your network credentials and broker details. It is recommended to use a `secrets.h` file. You can find an example under `./config/secrets.example.h.` The code below will receive messages from any source (If directed to the board) and print them out, and will send `Hello World` to the Group (All).

```cpp
#include <MiniMessenger.h>

// Configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* pass = "YOUR_WIFI_PASSWORD";
const char* broker = "BROKER_IP_OR_HOSTNAME";
const uint16_t port = 1883;
const char* group = "1";
const char* board = "1";

MiniMessenger messenger;

// Callback function for incoming messages
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  Serial.print("Message from ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  
  for (size_t i = 0; i < length; i++) {
    Serial.write(payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  // Initialize messenger
  messenger.onMessage(onMessage);
  messenger.begin(ssid, pass, broker, port, group, board);
}

void loop() {
  // Keep connections alive and poll for messages
  messenger.loop();

  // Example: Send a message every 5 seconds
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 5000) {
    messenger.sendToGroup("Hello World!");
    lastSend = millis();
  }
}
```
Please see the example code that shows how to parse a specific type of message received from the server.

## API Reference

### Initialization (Messenger)
- `bool begin(ssid, password, brokerHost, brokerPort, groupId, boardId)`: Connects to WiFi and MQTT.
- `void loop()`: Must be called in the main `loop()` to process messages and maintain connections.

### Sending Messages
- `bool sendToBoard(targetBoardId, text)`: Send a string to a specific board.
- `bool sendToGroup(text)`: Broadcast a string to everyone in your group.
- `bool sendToBoard(targetBoardId, data, length)`: Send raw bytes to a specific board.

### Receiving Messages
- `void onMessage(callback)`: Register a function to handle incoming messages. The callback receives `MessageMetadata` (containing `fromBoardId`, `groupId`, etc.) and the payload.

### Connection Status
- `bool isConnected()`: Returns `true` if both WiFi and MQTT are connected.


## Protocol Fundamentals

### Message Format
All messages are sent as simple text strings using **Key=Value** pairs separated by spaces.
*   **Case Sensitivity:** Keys (like `type=`) are case-sensitive.
*   **No Spaces in Values:** Values must not contain spaces (use underscores if needed).
*   **Example:** `type=isFertile tag_id=A1B2C3D4 board_id=2`

### Core Identification Fields
Every message includes these identifiers to ensure the server routes data correctly:
1.  **`type`**: The message category (e.g., `heartbeat`, `register`).
2.  **`team_id`**: Your assigned team number (e.g., `12`).
3.  **`board_id`**: Your specific Robot name (e.g. `Wilson`, `BoxOnWils`).

---

## Message Dictionary

### A. Robot to Server (Uplink)

These messages can change before the challenge finals (unlikely), please refer to the table below to keep your functions up-to-date. Any changes will be communicated through Moodle as well.

| Type | Example Payload | Description |
| :--- | :--- | :--- |
| `register` | `type=register team_id=5 board_id=1` | **Mandatory.** Must be sent every ~10s to stay "Online". |
| `isFertile` | `type=isFertile tag_id=ABC12345 board_id=1` | Sent when scanning an RFID tag to check planting status. |
| `seedPlanted`| `type=seedPlanted tag_id=ABC12345 board_id=1`| Sent after planting to update the server map. |
| `openAirlock`| `type=openAirlock airlock=A tag_id=ABC12345 board_id=1` | Request entry/exit permission. A or B on airlock. **Requires tag_id.** |
| `reviveRequest`| `type=reviveRequest target_team=12 target_board=1`| Revive a stranded robot (must be adjacent). |
| `getMap` | `type=getMap board_id=1` | Request a direct binary map refresh. |

### B. Server to Robot (Downlink)

| Type | Example Payload | Description |
| :--- | :--- | :--- |
| `heartbeat` | `type=heartbeat enable=1 seq=100 time_left=115` | `enable=1` allows movement; `0` forces stop. Time left on your run.|
| `disable` | `type=disable enabled=false reason=stranded` | Robot is stranded/disabled until revived. |
| `isFertileReply`| `type=isFertileReply fertile=true planted=false x=6 y=7` | Position and fertility response. |
| `emergency` | `type=emergency enabled=true` | **Global Stop.** Triggered by the Big Red Button. Stop immediately! |
| `Grid Map` | `[Raw Binary 21 Bytes]` | **Global Topic.** Full 2-bit field state. |
| `Distress Alert`| `type=distress count=1 robot0=12.1,5,3` | **Global Topic.** Information on stranded robots. |
| `openAirlockReply`| `type=openAirlockReply airlock=A accepted=true queue_enter=0 queue_exit=0` | Confirmation of airlock request. If `accepted=false`, a `reason` (e.g., `queue_full`) is included. |
| `reviveReply` | `type=reviveReply status=success target=1` | Confirmation of a successful rescue. |

---
## Distinguishing Global & Team Messages
### A. Team Status (Team Topic - 6 Bytes)
The server broadcasts this 6-byte payload every second. Use it to coordinate movements with your teammates.

| Byte | Field | Description |
| :--- | :--- | :--- |
| **0** | `queueExit` | Number of robots waiting to exit (Airlock B). |
| **1** | `airlockBBusy` | `1` if the Exit door is currently opening/closing. |
| **2** | `queueEnter` | Number of robots waiting to enter (Airlock A). |
| **3** | `airlockABusy` | `1` if the Entrance door is currently opening/closing. |
| **4** | `emergency` | `1` if the team is in an Emergency stop state. |
| **5** | **`reEntryRequested`** | **`1` if a robot has scanned the "Request Re-entry" tag.** |

**Arduino Example:**
```cpp
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    // 1. Check for Team Status Broadcast
    if (length == 6) {
        bool reEntryFlag = (payload[5] == 1);
        if (reEntryFlag) Serial.println("Base Re-entry Requested!");
        return;
    }
    
    // 2. Check for Occupancy Map
    if (length == 21) {
        // ... handle map
        return;
    }

    // 3. Normal text handling...
}
```
### B. Global Topic (21 Bytes or Text)
Since the Occupancy Map and Distress Alerts both arrive on the **Global Broadcast** topic, use this logic to tell them apart:

1.  **Check Length:** If the message is exactly **21 bytes**, it is the binary Occupancy Map.
2.  **Check Content:** If it starts with `type=distress`, it is a text-based distress signal.

---

## Occupancy Map Protocol (lab/map/grid)

The server broadcasts the full field state every 5 seconds (and instantly on changes). This is a raw binary payload, **not** text. You can also request an immediate update by sending `type=getMap board_id=1` (the server will reply directly to your board with the same binary payload).

**Example Payload (Hex Representation):**
A completely unexplored map (Fog of War) consists of 81 cells all set to state `3` (`11` in binary). This results in 21 bytes of data that looks like this in memory:
`FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 03`

### 2-Bit Cell States
Each cell in the 9x9 grid uses 2 bits to define its status. This protocol implements a **Fog of War** mechanic; robots only see the state of cells that have been "Explored" by any robot scanning them.

*   **`00` (0): Sterile** - Explored and confirmed as non-fertile.
*   **`01` (1): Fertile** - Explored and available for planting.
*   **`10` (2): Seeded** - Explored and already occupied by a seed.
*   **`11` (3): Unexplored** - Fog of War. The status of this cell is unknown until a robot visits it.


### Contributing and Linting

To ensure this library remains compatible with the official Arduino Library Manager, this repository uses **Arduino Lint** via GitHub Actions. 

Whenever you push code or open a Pull Request, our automated workflow will check the repository structure, `library.properties` file, and code styling to ensure it meets strict Arduino Registry compliance. Please ensure the Arduino Lint badge is passing (green) before requesting a merge!
