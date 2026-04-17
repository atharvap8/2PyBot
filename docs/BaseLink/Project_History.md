# Project History and Development Log

This document meticulously records the evolutionary timeline of the Self-Balancing Robot project, detailing the engineering hurdles encountered dynamically navigating complex physics simulations.

## Phase 1: Foundational Locomotion
**Objective:** Validate fundamental Stepper Motor execution parameters integrating TMC2208 drivers identically through ESP32 silicon boundaries natively.
- **Initial Methodology:** Leveraged external standard generic hardware libraries executing simplistic high-latency loop timing constructs natively. 
- **Result:** Hardware manifested aggressively jittery characteristics definitively caused asynchronously by conflicting CPU cache delays structurally starving motor pulse pipelines.
- **Refinement:** The core team implemented completely custom low-level algorithmic hardware timer bindings (`stepper_control.h`), forcibly relegating pulse generation entirely outside mainline operations directly towards the structural Silicon `IRAM` interrupts generating pristine 20kHz timing configurations.

## Phase 2: Sensor Calibration & Initial Balance Profiles
**Objective:** Structure pristine geometric feedback explicitly measuring absolute horizontal tilt offsets identically maintaining a centralized gravity column globally. 
- **Initial Methodology:** A completely generic Complimentary Filter executed statically against fundamental Accelerometer readings structurally isolated from active filtering structures natively. 
- **Result:** Extreme baseline integration issues manifested immediately causing robotic operations theoretically mathematically sound to execute completely erratic trajectory profiles due specifically to gyroscopic ambient drift constants fundamentally escalating unmeasured.
- **Refinement:** Developed comprehensive Mahony AHRS algorithms actively extracting precise geometric coordinate orientation explicitly integrating magnetometer validations ensuring complete immunity to mechanical vibration parameters structurally.

## Phase 3: The Cascaded PID & Velocity Problem
**Objective:** Initiate algorithmic loop constraints directly isolating identical velocity configurations natively matching physical displacement realities inherently measured.
- **Initial Methodology:** Dual-Cascaded loops executing isolated velocity checks structurally dominating identical fundamental angular matrices.
- **Result:** Robotic architecture executed incredibly well initially globally, but failed catastrophically immediately when directed using aggressive external steering commands logically structurally. The delay generated across independent PID states fundamentally manifested actively propagating extreme Phase-Lag.
- **Refinement:** Annihilated the Velocity PID structure entirely globally. Consolidated logic executing specifically relying exclusively on high-speed Proportional derivatives identically matching "Ghost Target" driving mechanics explicitly mapped avoiding structural acceleration spikes natively. 

## Phase 4: ESP-NOW Peripheral Telemetry Binding
**Objective:** Incorporate precise joystick manipulation physically translating intent without requiring absolute WiFi/TCP initialization inherently prone to transmission delays natively disrupting 200 Hz PID processing limitations completely.
- **Refinement:** Designed parallel decoupled hardware topologies identically maintaining separate codebases natively (`Joystick_Transmitter`). Validated strict UDP `__attribute__((packed))` constructs natively executing identically maintaining 50Hz rigid transmission speeds identically. Modified underlying ESP32 definitions successfully adapting architectural `esp_mac.h` integrations exclusively supporting the newly released native ESP32 Core 3.0 / IDF 5.0 libraries fundamentally mapping MAC derivations.

---

*Evolutionary development structurally concluded. Core fundamental locomotion physics identical logic paths strictly validated completely.*
