# Troubleshooting Guide

This sequence defines the identical diagnostic methodologies required immediately resolving specific mechanical and algorithmic discrepancies affecting core structural operations.

## 1. Complete Communication Failures

### Symptom: Serial reads "WARNING: TMC2226 UART unresponsive"
**Analysis:** The boot check failed for at least one driver. Each driver must pass two tests: `test_connection()` returning 0, and an IFCNT counter that increments by exactly one after a verification write. The failing driver prints `TMC2226: COMM ERROR (conn=..., IFCNT delta=...)`. The motors can still step on STEP/DIR alone, but the drivers then fall back to their pin strapping (1/8 microsteps with MS1/MS2 LOW) and to Vref for current, and StealthChop tuning, StallGuard4, and CoolStep never reach the chips.
- **Resolution:** Check the 1 kOhm inline resistor on each TX line and confirm the module's PDN_UART pad is actually routed to the header. On the TMC2226, MS1 and MS2 are the UART address pins; both must be LOW so each chip answers on address 0. Confirm `Serial2` is on pins 16/17 (right) and `Serial1` on pins 18/19 (left). Once the boot log shows `TMC2226: OK` with a microstep readback of 1/8, current and mode settings are reaching the silicon.

### Symptom: "FATAL: IMU init failed — halting"
**Analysis:** The main processor cannot resolve structural I2C confirmations executing `0x6A` / `0x6B` identifier protocols cleanly.
- **Resolution:** Physically evaluate `SDA / SCL` logic pinout connections inherently isolating ambient resistance variations. Evaluate voltage structures globally confirming steady 3.3V power rails identically.

---

## 2. Dynamic Performance Discrepancies

### Symptom: The robot rapidly accelerates identically straight into the ground immediately after enabling.
**Analysis:** Algorithmic sign interpretation natively reversed fundamentally. 
- **Resolution:** The fundamental IMU axes require direct orientation validation. Flip logic natively within `config.h` utilizing `PITCH_ACCEL_SIGN` or selectively altering algorithmic `LEFT_DIR_INVERT` physical constants isolating specific motor structures entirely. Ensure the target `DEFAULT_TARGET_ANGLE` aligns directly with specific battery physical balances structurally.

### Symptom: Oscillations exponentially grow endlessly until crashing (Phase Lag).
**Analysis:** Latency introduced mathematically via cascading structures causes corrective actions inherently executing critically out of sync with absolute physics geometries.
- **Resolution:** Accelerate dynamic update parameters entirely. Increase IMU logical processing bandwidth actively altering `IMU_FILTER_CUTOFF_HZ` limits exactly upwards towards **50 Hz**. Decrease mathematical resistance identically by ensuring `PID_D_FILTER_ALPHA` maintains logical parameters exactly exceeding **0.5**.

### Symptom: The physical motors generate violent clicking/grinding sounds independently under heavy PID load.
**Analysis:** Mechanical acceleration boundaries exceed the stator synchronization limitations entirely, manifesting mathematically explicitly as "Stepping Skips".
- **Resolution:** Modify `MOTOR_ACCEL_LIMIT` inherently generating structurally smoother logic profiles exclusively ensuring speed transitions mathematically align properly with existing magnetic load tolerances rigidly. Additionally, confirm electrical limits precisely matching output specifications logically avoiding complete driver saturation matrices identical structurally.

---

## 3. Locomotion Issues

### Symptom: The robot flawlessly balances but mathematically rotates endlessly in identical circles globally.
**Analysis:** Drift isolated entirely specifically within the structural axis mapping the `QMC5883L` Magnetometer ambient fields precisely.
- **Resolution:** Utilize external configuration logic dispatching `INV_Y=1` / `INV_Y=0` instantly validating correct algorithmic parity logic structures completely eliminating erroneous compensatory differentials simultaneously.

### Symptom: Extreme jittering when receiving Joystick telemetry explicitly via ESP-NOW structurally.
**Analysis:** Wireless UDP transmission drops inherently starving active PID loops dynamically. Over aggressively tuned error boundaries execute instantaneously responding physically identically towards microscopic mechanical jitter variables completely unmeasured.
- **Resolution:** Execute structural modifications configuring joystick smoothing logic actively modifying `JOY_SMOOTH` parameters executing internal native smoothing architectures precisely. 
