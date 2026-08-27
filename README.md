# 2PyBot: Self-Balancing Robot Project

[![Status](https://img.shields.io/badge/status-active-brightgreen)](#)
[![MCU](https://img.shields.io/badge/MCU-ESP32-blue)](#)
[![Wireless](https://img.shields.io/badge/Wireless-ESP--NOW-orange)](#)
[![Language](https://img.shields.io/badge/Firmware-Arduino%20C%2B%2B-lightgrey)](#)
[![Python](https://img.shields.io/badge/Dashboard-Python%203-yellow)](#)
[![Release](https://img.shields.io/github/v/release/atharvap8/2PyBot?label=release&color=success)](https://github.com/atharvap8/2PyBot/releases)
[![Firmware CI](https://github.com/atharvap8/2PyBot/actions/workflows/firmware-ci.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/firmware-ci.yml)
[![Python Lint](https://github.com/atharvap8/2PyBot/actions/workflows/python-lint.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/python-lint.yml)
[![Docs Check](https://github.com/atharvap8/2PyBot/actions/workflows/docs-check.yml/badge.svg)](https://github.com/atharvap8/2PyBot/actions/workflows/docs-check.yml)

2PyBot is a self-balancing wheeled robot built around the ESP32 microcontroller. It uses stepper motors for precise actuation, a 6-axis IMU fused with a Mahony AHRS filter for orientation estimation, and an ESP-NOW wireless link to a custom joystick transmitter. A Python desktop dashboard provides real-time telemetry and live PID tuning over Bluetooth.

---

## Project Structure

```
2PyBot/
├── firmware/
│   ├── BaseLink/               # Robot (receiver) firmware
│   │   ├── BaseLink.ino
│   │   ├── config.h
│   │   ├── imu_sensor.h / .cpp
│   │   ├── stepper_control.h / .cpp
│   │   ├── bt_gamepad.h
│   │   ├── led_ring.h / .cpp
│   │   └── system_architecture.md  # Inline architecture reference
│   └── Controller/             # Joystick transmitter firmware
│       └── Controller.ino
├── gui/                        # Desktop control & telemetry software
│   └── robot_controller_ui.py
├── hardware/                   # PCB schematics and BOM (KiCad)
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

Motor stepping is handled entirely in a **20 kHz hardware timer ISR**, independent of the main loop. This produces jitter-free, precise pulses to the TMC2226 stepper drivers without relying on `delay()` or blocking calls. Bresenham-style accumulators are used to generate non-integer step rates accurately.

The TMC2226 drivers run in UART mode. At boot, the firmware programs the RMS current, the 1/8 microstep resolution, and the chopper settings over a dedicated hardware UART per driver, then verifies each write with the chip's IFCNT counter. The drivers operate in StealthChop, which enables two features the previous TMC2208s lacked: StallGuard4 load measurement with hardware stall flags, and CoolStep load-adaptive current scaling. Motion itself is plain STEP/DIR from the ISR; the UART carries only configuration and diagnostics.

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
| `BaseLink.ino` | Main entry point; loop timing and state orchestration |
| `config.h` | All tunable constants: PID gains, pin assignments, motor specs |
| `imu_sensor.h / .cpp` | IMU initialization, DMP/raw read, Mahony filter integration |
| `stepper_control.h / .cpp` | ISR-driven step generation, velocity ramping, direction control |
| `bt_gamepad.h` | Bluepad32 gamepad link; pairs the EVOFOX pad directly to the robot |
| `led_ring.h / .cpp` | WS2812 expression ring driven over the ESP32 RMT peripheral |
| `gui/robot_controller_ui.py` | Python telemetry dashboard (PySerial + Matplotlib) |
| `tools/tuner/tuner.html` | Web Bluetooth tuner, no installation required |

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
| [TMC2226 Migration](docs/BaseLink/TMC2226_Migration.md) | Driver swap from TMC2208 to TMC2226: wiring, firmware, and tuning notes |
| [Controller Architecture](docs/Controller/System_Architecture.md) | Transmitter packet structure and joystick mapping |

---

## Getting Started

### Robot Firmware

1. Install the **esp32_bluepad32** board package. The board manager URL and setup steps are in `firmware/BaseLink/README.md`. The robot firmware needs this package for the Bluetooth gamepad and will not compile on the stock ESP32 core.
2. Open `firmware/BaseLink/BaseLink.ino` in the Arduino IDE and select `ESP32 Dev Module` from the **ESP32 + Bluepad32 Arduino** section, with the **Huge APP (3MB)** partition scheme.
3. Review `config.h` and set pin assignments and motor parameters for your hardware.
4. Install the required libraries: TMCStepper, STM32duino ISM6HG256X, QMC5883LCompass, and NeoPixelBus by Makuna.
5. Flash to the robot ESP32 and confirm the boot log shows `TMC2226: OK` for both drivers.

### Controller Firmware

1. Open `firmware/Controller/Controller.ino`.
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
- TMC2226 stepper motor drivers (×2, 2.0 A RMS, UART mode, StallGuard4 and CoolStep)
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
