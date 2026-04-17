# Program Flow and State Machine Architecture

This document meticulously traces the chronological structure spanning the initialization, evaluation, and physical execution routines inherent to the `Self_Balancing_Robot.ino` firmware.

## Process Initialization & Calibration Flow

Upon triggering the ESP32 microcontroller's `setup()` environment, sequential validation routines operate linearly to bootstrap isolated complex hardware peripherals.

```mermaid
sequenceDiagram
    participant OS as ESP32 Core
    participant IMU as ISM6HG256X (I2C)
    participant STEP as TMC2208 Steppers
    participant NET as ESP-NOW & BT

    OS->>IMU: Configure High-Performance Registers (400kHz)
    IMU-->>OS: Confirm IDs and Validate Full Scales
    OS->>STEP: Map UART Pins / Setup 20kHz Alarm Timer
    OS->>OS: delay(STARTUP_SETTLE_MS) enforces thermal settling
    OS->>IMU: calibrateGyro() executing 500 static readings
    OS->>NET: Initiate ESP-NOW and SPP Bluetooth Stacks
    OS->>OS: Assert STATE_IDLE configuration
```

## The State Machine (Finite Automaton)

The central operational methodology explicitly prevents erroneous code from activating powerful steppers without specific conditions being flawlessly met.

```mermaid
stateDiagram-v2
    [*] --> STATE_INIT : Hardware Boot
    
    STATE_INIT --> STATE_IDLE : Boot Success
    
    STATE_IDLE --> STATE_BALANCING : Drive Enabled ('E') + Robot is Level (< 5°)
    
    STATE_BALANCING --> STATE_IDLE : Drive Disabled ('X')
    
    STATE_BALANCING --> STATE_FALLEN : Fatal Angle Exceeded (> MAX_TILT_ANGLE)
    
    STATE_FALLEN --> STATE_IDLE : Disabled
    STATE_FALLEN --> STATE_BALANCING : Robot explicitly placed Level (< MAX_TILT_ANGLE) + 'E' Reasserted
```

### 1. `STATE_INIT`
- Confirms logic availability exclusively. Motors are logically and physically disconnected.
- Calibrates gyroscopic zeroes assuming flawless rigid stillness.

### 2. `STATE_IDLE`
- Dispatches identical diagnostic flashes utilizing the `ONBOARD_LED` representing logical wait status.
- Evaluates positional drift inputs and strictly forces internal compensators identically towards theoretical geometric zero offsets.

### 3. `STATE_BALANCING`
- Executes the entirety of the rigid `LOOP_FREQ_HZ` (200 Hz) mathematical hierarchy.
- Scales visual LED pulse delays proportionally mapping mathematical failure proximities (flashing quicker when structural strain escalates).
- Identifies independent external trajectory requests mapped perfectly against local acceleration physics generating kinetic torque.

### 4. `STATE_FALLEN`
- Safety override triggered actively upon exceeding structurally irrecoverable deviation angles (e.g., $95.0^{\circ}$).
- Slashes identical acceleration limits instantly resolving physical logic gates mapping the Motor `EN` pins directly HIGH.
- Completely locks identical evaluation until specific human interference physically centers the chassis directly perpendicular relative to local gravity, simultaneously resubmitting explicit 'Enable' commands natively via wireless diagnostics.
