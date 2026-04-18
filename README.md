# 2PyBot — Self-Balancing Robot Platform

[![Status](https://img.shields.io/badge/status-active-brightgreen)](#)
[![MCU](https://img.shields.io/badge/MCU-ESP32-blue)](#)
[![Wireless](https://img.shields.io/badge/Wireless-ESP--NOW-orange)](#)
[![Language](https://img.shields.io/badge/Firmware-Arduino%20C%2B%2B-lightgrey)](#)
[![Python](https://img.shields.io/badge/Dashboard-Python%203-yellow)](#)

2PyBot is a self-balancing wheeled robot built around the ESP32 microcontroller. It uses stepper motors for precise actuation, a 6-axis IMU fused with a Mahony AHRS filter for orientation estimation, and an ESP-NOW wireless link to a custom joystick transmitter. A Python-based desktop dashboard provides real-time telemetry and live PID tuning over Bluetooth.

---

## Project Structure

```
24_2PyBot/
├── firmware/
│   ├── BaseLink/               # Robot (receiver) firmware
│   │   ├── Self_Balancing_Robot.ino
│   │   ├── config.h
│   │   ├── imu_sensor.h / .cpp
│   │   ├── stepper_control.h / .cpp
│   │   ├── pid_controller.h
│   │   ├── espnow_comm.h
│   │   ├── serial_tuner.h
│   │   ├── robot_controller_ui.py
│   │   └── tuner.html
│   └── Controller/             # Joystick transmitter firmware
│       └── Joystick_Transmitter.ino
├── hardware/                   # PCB schematics and BOM (KiCad)
├── software/                   # Auxiliary desktop tools and scripts
├── models/                     # 3D CAD files (STEP / GLB)
├── assets/                     # Media: demo videos, photos, renders
├── docs/
│   ├── BaseLink/               # Robot-side technical documentation
│   │   ├── System_Architecture.md
│   │   ├── PID_Theory_and_Math.md
│   │   ├── Config_and_Tuning_Guide.md
│   │   ├── Troubleshooting_Guide.md
│   │   ├── Program_Flow_State_Machine.md
│   │   └── Project_History.md
│   └── Controller/             # Transmitter-side documentation
│       └── System_Architecture.md
└── README.md
```

---

## System Architecture

### Balancing Control Loop

The main control loop runs at **200 Hz** (`dt = 5 ms`). On each tick:

1. The IMU (ISM6HG256x) is read and fused through a **Mahony AHRS** filter to produce a drift-corrected pitch angle.
2. A **Balance PID** controller computes a target stepper speed proportional to the deviation from the equilibrium angle.
3. An optional **Heading PID** loop corrects yaw drift using magnetometer feedback (QMC5883L).

### Stepper Actuation

Motor stepping is handled entirely in a **20 kHz hardware timer ISR**, independent of the main loop. This produces jitter-free, precise pulses to the TMC2208 stepper drivers without relying on `delay()` or blocking calls. Bresenham-style accumulators are used to generate non-integer step rates accurately.

### Wireless Control

The **Controller** firmware runs on a second ESP32 fitted with a joystick and buttons. It transmits a structured packet over **ESP-NOW** at low latency to the robot. The packet encodes a target pitch offset (in degrees), allowing the robot's control law to translate operator intent directly into a lean angle.

### Telemetry & Tuning

The robot streams telemetry packets over Bluetooth serial. Two interfaces are provided for interaction:

- **`robot_controller_ui.py`** - A Python desktop dashboard for real-time telemetry plots and control input.
- **`tuner.html`** - A browser-based Web Bluetooth interface for on-the-fly PID constant adjustment.

---

## Firmware Modules

| File | Role |
| :--- | :--- |
| `Self_Balancing_Robot.ino` | Main entry point; loop timing and state orchestration |
| `config.h` | All tunable constants: PID gains, pin assignments, motor specs |
| `imu_sensor.h / .cpp` | IMU initialization, DMP/raw read, Mahony filter integration |
| `stepper_control.h / .cpp` | ISR-driven step generation, velocity ramping, direction control |
| `pid_controller.h` | Generic PID template used by balance and heading loops |
| `espnow_comm.h` | ESP-NOW peer registration, packet parsing, and callback handler |
| `serial_tuner.h` | Bluetooth serial command parser for live PID adjustment |
| `gui/robot_controller_ui.py` | Python telemetry dashboard (PySerial + Matplotlib) |
| `tuner.html` | Web Bluetooth PID tuner, no installation required |

---

## Documentation

All technical write-ups are in [`docs/`](docs/).

| Document | Description |
| :--- | :--- |
| [System Architecture](docs/BaseLink/System_Architecture.md) | Control loop design, module interactions, and data flow |
| [PID Theory and Math](docs/BaseLink/PID_Theory_and_Math.md) | Derivation and implementation of the balance and heading loops |
| [Config and Tuning Guide](docs/BaseLink/Config_and_Tuning_Guide.md) | Step-by-step instructions for calibrating and tuning gains |
| [Program Flow & State Machine](docs/BaseLink/Program_Flow_State_Machine.md) | Boot sequence, operational states, and fault handling |
| [Troubleshooting Guide](docs/BaseLink/Troubleshooting_Guide.md) | Diagnosis of common issues: oscillation, drift, crashes |
| [Project History](docs/BaseLink/Project_History.md) | Revision log and design evolution |
| [Controller Architecture](docs/Controller/System_Architecture.md) | Transmitter packet structure and joystick mapping |

---

## Getting Started

### Robot Firmware

1. Open `firmware/BaseLink/Self_Balancing_Robot.ino` in the Arduino IDE.
2. Review `config.h` and set pin assignments and motor parameters for your hardware.
3. Install required libraries (see `config.h` header comments for the full list).
4. Flash to the robot ESP32.

### Controller Firmware

1. Open `firmware/Controller/Joystick_Transmitter.ino`.
2. Set the robot's MAC address in the ESP-NOW peer configuration.
3. Flash to the transmitter ESP32.

### Python Dashboard

```bash
cd gui
pip install pyserial matplotlib
python robot_controller_ui.py
```

Pair the robot over Bluetooth before launching the dashboard. The COM port can be configured at the top of the script.

---

## Hardware

PCB design files (KiCad) and the bill of materials are located in [`hardware/`](hardware/). 3D printable chassis files are in [`models/`](models/).

**Key components:**

- ESP32 (×2 — robot and transmitter)
- TMC2208 stepper motor drivers (×2)
- NEMA 17 stepper motors (×2)
- ISM6HG256x IMU breakout (Custom PCB)
- QMC5883L magnetometer
- MP1584 Buck Converter (12V-5V)
- 3S Li-Ion Battery
- XT-60 Male-Female Pair
- Slide Switch
- Standoffs, Screws
- Zero PCB
- Nema17 Motor Bracket
- Robot Wheels (Big diameter is better)

---

## Media

Demo footage and build photos are in [`assets/`](assets/).

| Asset | Description |
| :--- | :--- |
| [Demo Video](assets/demo.mp4) | Robot balancing and driving under remote control |
| [Build Photo](assets/build.jpg) | Assembled hardware with electronics visible |
| [Dashboard Screenshot](assets/dashboard.png) | Python telemetry dashboard during a live session |

---

*Self-balancing robotics project — open for reference and research use.*
