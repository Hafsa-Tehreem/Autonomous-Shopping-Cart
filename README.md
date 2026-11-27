# Autonomous Grocery-Picking Mobile Robot

An end-to-end autonomous warehouse and grocery fulfillment robot built on an **Arduino Mega 2560**, **ESP32-CAM**, **Pololu encoder gearmotors**, a **DWIN 4.3" HMI touchscreen**, a **rotating robotic pick-and-place arm**, and a **Python computer-vision & path-planning controller**.

Users select grocery items on the DWIN touchscreen interface (or via the Python console), after which the robot calculates the shortest obstacle-free path through a mapped aisle grid, navigates using closed-loop wheel encoders, verifies each target item via **ArUco fiducial markers**, picks up the item using its onboard robotic arm, rotates to deposit it into a **self-contained rear storage container**, and returns to its docking station.

---

## System Architecture

```text
┌─────────────────────────┐     Hardware Serial1 (UART)   ┌─────────────────────────┐
│  DWIN 4.3" HMI Display  │ ────────────────────────────► │                         │
│ (DMG48270C043_04WTC)    │                               │    Arduino Mega 2560    │
└─────────────────────────┘                               │  • Pololu Encoder ISR   │
                                                          │  • L298N Motor Control  │
┌─────────────────────────┐       Wi-Fi (SoftAP HTTP)     │  • Robotic Arm & Gripper│
│   Python Brain (PC)     │ ◄───────────────────────────► │                         │
│  • BFS Path Planner     │       ┌───────────────┐       │                         │
│  • OpenCV ArUco Scanner │ ────► │   ESP32-CAM   │ ────► │                         │
└─────────────────────────┘       │ (192.168.4.1) │Serial2└─────────────────────────┘
                                  └───────────────┘
```

### End-to-End Workflow
1. **Order Entry (HMI / Console):** The user selects desired grocery items via the DWIN 4.3" touchscreen (`DMG48270C043_04WTC`)[cite: 1] or enters them into the Python controller.
2. **Shortest-Path Planning:** Python runs a **Breadth-First Search (BFS)** algorithm on the 2D aisle grid (`0` = open aisle, `1` = shelf/obstacle) to compute the shortest collision-free route to each item's aisle stop point.
3. **Encoder-Guided Navigation:** Movement commands (`F`, `B`, `L`, `R`, `S`) are transmitted over Wi-Fi to the ESP32-CAM (`192.168.4.1`) and relayed via dedicated hardware UART (`Serial2`) to the Arduino Mega. Hardware interrupts count quadrature pulses from the **Pololu metal gearmotors** for accurate distance and 90° pivot turns.
4. **ArUco Visual Verification:** Upon reaching the target shelf, the robot rotates toward the shelf and streams live MJPEG video from the ESP32-CAM (`:81/stream`) to OpenCV, scanning for the item's unique `DICT_4X4_50` ArUco marker ID.
5. **Automated Pick & Rear Container Stowage:** Once the ArUco tag is verified, the onboard **robotic arm** extends to grasp the scanned item, lifts and rotates 180° toward the rear of the chassis, and releases the item into the **self-contained onboard cargo bin**.
6. **Multi-Item Fulfillment & Auto-Docking:** Steps 2–5 repeat for all remaining items on the list before the robot navigates back to the `(0, 0)` home docking station.

---

## Hardware Stack

| Component | Model / Specification | Role |
| :--- | :--- | :--- |
| **Main Controller** | Arduino Mega 2560 (ATmega2560) | Reads Pololu quadrature encoders via hardware interrupts, drives L298N motors, actuates pick-and-place arm, handles multi-UART communication (`Serial`, `Serial1`, `Serial2`) |
| **Vision & Wi-Fi Bridge** | AI-Thinker ESP32-CAM | Broadcasts `RC_Car_AP` Wi-Fi hotspot (`192.168.4.1`), streams live video, bridges HTTP commands to Serial |
| **Drive Motors** | Pololu Metal Gearmotors w/ Built-in Magnetic Encoders | 6-wire quadrature Hall-effect feedback for precise grid stepping and 90° turns |
| **Motor Driver** | L298N Dual H-Bridge | Drives left and right Pololu gearmotors from battery pack |
| **Manipulation & Payload** | Multi-DOF Servo Robotic Arm + Rear Container | Picks verified grocery items from shelves and rotates to place them into the rear onboard storage bin |
| **User Interface** | DWIN 4.3" HMI Touch Display (`DMG48270C043_04WTC`) | Commercial-grade touchscreen sending DGUS VP hex packets over TTL UART[cite: 1] |

---

## Pinout & Wiring Overview (Arduino Mega 2560)

Taking advantage of the Arduino Mega's multiple hardware UART ports eliminates the need for `SoftwareSerial` and keeps USB `Serial` (`TX0`/`RX0`) completely free for debugging and flashing.

### 1. Pololu Motors & L298N → Arduino Mega
* **L298N Inputs:** `IN1` → `D22`, `IN2` → `D23` (Left) | `IN3` → `D24`, `IN4` → `D25` (Right)
* **Left Pololu Encoder:** `OUT A` → `D2` (Interrupt 0), `OUT B` → `D4`, `VCC` → `5V`, `GND` → `GND`
* **Right Pololu Encoder:** `OUT A` → `D3` (Interrupt 1), `OUT B` → `D5`, `VCC` → `5V`, `GND` → `GND`

### 2. DWIN 4.3" HMI LCD → Arduino Mega (`Serial1`)
* **DWIN `TX2`** → Arduino Mega `D19` (`RX1`)
* **DWIN `RX2`** → Arduino Mega `D18` (`TX1`)
* **Power:** `5V` and common `GND`

### 3. ESP32-CAM → Arduino Mega (`Serial2`)
* **ESP32-CAM `U0T` (TX)** → Arduino Mega `D17` (`RX2`)
* **ESP32-CAM `U0R` (RX)** → Arduino Mega `D16` (`TX2`) *(Optional, for `DONE` acknowledgments)*
* **Power:** Regulated `5V` supply and common `GND`

### 4. Robotic Arm Servos → Arduino Mega
* **Base Rotation / Shoulder / Elbow / Gripper:** PWM Pins `D6`, `D7`, `D8`, `D9` (powered via an external 5V–6V UBEC/regulator sharing common `GND` with the Mega).

---

## Repository Structure

```text
├── arduino_mega_controller/
│   └── arduino_mega_controller.ino   # Encoder navigation, arm pick-and-place, & DWIN UART parser
├── esp32_cam_bridge/
│   └── esp32_cam_ap_bridge.ino       # SoftAP Wi-Fi server, MJPEG video stream, & HTTP-to-Serial bridge
├── python_brain/
│   └── grocery_ap_navigator.py       # BFS path planner, OpenCV ArUco detector, & task orchestrator
└── README.md
```

---

## Software Setup & Installation

### 1. Python Environment
Install the required computer-vision, HTTP, and serial communication libraries:
```bash
pip install opencv-contrib-python requests numpy pyserial
```
*(Note: `opencv-contrib-python` is required for `cv2.aruco` marker detection).*

### 2. Firmware Flashing
1. **ESP32-CAM:** Flash `esp32_cam_ap_bridge.ino` using the **AI Thinker ESP32-CAM** board profile in Arduino IDE.
2. **Arduino Mega 2560:** Select **Tools > Board > Arduino Mega or Mega 2560** and flash `arduino_mega_controller.ino`.

### 3. Running the System
1. Place the robot at grid coordinate **`(0, 0)`** facing **East** with its rear storage container empty and robotic arm in the stowed home position.
2. Connect your host computer's Wi-Fi to **`RC_Car_AP`** (Password: `password123`).
3. Ensure `DICT_4X4_50` ArUco markers (`ID 10, 20, 30, 40`) are affixed to the target grocery items on the shelves.
4. Launch the controller:
   ```bash
   python grocery_ap_navigator.py
   ```
5. Select grocery items via the DWIN touchscreen or terminal prompt to begin autonomous collection.

---

## Calibration Notes
* **Encoder Ticks (`TICKS_PER_CELL` & `TICKS_PER_90_TURN`):** Adjust in the Arduino Mega sketch based on your specific Pololu gearbox ratio (e.g., 50:1, 100:1, or 298:1) and wheel diameter.
* **Arm Kinematics & Gripper Angles:** Tune servo min/max pulse angles for shelf pickup height, gripper closure force, and the 180° rotation angle required to clear the chassis and drop items cleanly into the rear self-contained bin.
* **DGUS Baud Rate:** Verify whether your `DMG48270C043_04WTC` CFG file is configured for `115200` or `9600` baud to match `Serial1.begin()`.
