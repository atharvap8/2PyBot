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

## Phase 5: Actuation Upgrade, TMC2208 to TMC2226
**Objective:** Remove the two standing limits of the old driver setup: a UART link that never worked on this hardware, and a stall ceiling near 9000 pulses per second that forced a conservative `MAX_SPEED_STEPS` cap of 6000.
- **Initial Limitation:** The TMC2208 modules topped out near 1.4 A RMS, offered no load telemetry, and their UART reported COMM ERROR on this robot, so every configured value silently failed to reach the chips. The drivers ran on pin strapping and Vref alone.
- **Refinement:** Both axes moved to TMC2226 drivers, driven through the register-compatible `TMC2209Stepper` class at UART address 0 (MS1 and MS2 strapped LOW on both modules). The UART link was repaired and is now verified at every boot with an IFCNT write check, so configuration failures cannot hide. The chopper runs in StealthChop, which StallGuard4 and CoolStep both require; the stall ceiling was measured identical to spreadCycle, about 9000 microsteps per second at 1100 mA, and testing showed the limit is set by the motor and supply voltage, not the driver. StallGuard4 thresholds were tuned on the hardware (`SGTHRS_LEFT 76`, `SGTHRS_RIGHT 77`) and CoolStep runs with a half-current floor above roughly 1250 microsteps per second. Run current rose from 1100 mA to 1500 mA and `MAX_SPEED_STEPS` from 6000 to 8500. Microstepping stays at 1/8, now programmed over UART with internal interpolation to 1/256, so `STEPS_PER_M` and every tuned gain carried over unchanged. The full delta is documented in `TMC2226_Migration.md`.

---

*Evolutionary development structurally concluded. Core fundamental locomotion physics identical logic paths strictly validated completely.*
