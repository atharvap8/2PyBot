# Configuration and Tuning Guide

Achieving stable locomotion on an inverted two-wheel pendulum involves navigating a highly complex interplay of mechanical lag, digital filter latency, and aggressive torque delivery. This guide documents exactly how to configure the physics model utilizing `serial_tuner` and `config.h`.

## 1. Preparing the Constants (`config.h`)

Before initiating live tuning procedures, ensure physical limits match your mechanical hardware identically:

- `TIMER_FREQ_HZ (20,000)`: Defines the maximum theoretical hardware stepping frequency. Altering this breaks discrete pulse timing matrices severely unless compensated across the structural loop geometry globally.
- `MOTOR_ACCEL_LIMIT (200000.0f)`: Defines mathematical max capabilities explicitly mapped to motor stator torque constraints. If motors visually "stutter" instead of screaming when accelerating intensely, lower this instantly to prevent electromagnetic saturation and stall conditions.
- `DEFAULT_TARGET_ANGLE (-1.469f)`: This represents absolute mathematical true horizontal specifically respecting internal chassis battery imbalances securely.

---

## 2. PID Tuning Strategy

Live parameter injection prevents destructive compile cycles. Connect via Bluetooth Terminal (`Self_Balancing_Robot`) or native USB Serial, and adhere strictly to this methodology:

### Step 1: Isolating Proportional Action (Kp)
1. Initialize structure exclusively resetting Integral and Derivative loops:
   - `I=0`, `D=0`, `P=100`
2. Power the hardware (`E`) whilst gripping the chassis loosely to prevent explosive torque.
3. Systematically scale Kp upwards logically until the absolute physical manifestation reflects intense, uniform oscillatory oscillation without structural degradation.
4. Scale structurally downward precisely roughly 15%. This determines the permanent Proportional Baseline Matrix.

### Step 2: Smoothing with Derivative Factor (Kd)
1. Introduce minor variables (e.g. `D=1.0`).
2. Gently evaluate external disturbance resilience physically nudging the structure. 
3. Excessive scaling logic explicitly induces catastrophic chassis shivering due to high-frequency IMU electromagnetic data multiplication. 
4. Scale up gently exclusively until low-frequency oscillatory behaviors decay flawlessly, completely mitigating physical overshoot properties native natively across balancing thresholds.

### Step 3: Drift Cancellation (Ki)
1. Constant static battery weight discrepancies predictably force identical robotic frames to travel eternally backwards or forwards seeking false-centers.
2. Incorporate explicitly diminutive constants (e.g. `I=2.0`).
3. Ensure absolute strict mathematical enforcement limiting integral windups mapping variables securely within `config.h` via `INTEGRAL_LIMIT` constants.

---

## 3. Advanced Structural Enhancements

### D-Term LPF Management (`PID_D_FILTER_ALPHA`)
Derivative variables exaggerate minor angular imperfections rapidly.
- Adjust `PID_D_FILTER_ALPHA` dynamically within `config.h`.
- `< 0.2` completely paralyzes reactive behaviors causing catastrophic phase-lag collapse. 
- `1.0` introduces excessive grinding noises entirely originating directly from motor stators mimicking noise profiles precisely.
- Typical structural optimum logic evaluates exactly near **0.7**.

### Position Odometry Overrides (`PKP` and `PKD`)
Actuating precise positional limits necessitates defining `PKP`.
- Transmit `PKP=0.0006` dynamically activating algorithmic pushback protocols explicitly measuring cumulative uncommanded travel displacement structurally.
- Transmit `EN_P=0` totally disabling rigid odometric constraints executing solely standard gravity modeling.

## 4. Live Command Lexicon
Execute immediate configurations observing exact upper-case syntactical limits:

| Command | Action | Impact |
|:---:|:---|:---|
| **`P=X`** | Sets $K_p$ | Immediate reaction strength. |
| **`D=X`** | Sets $K_d$ | Damping influence and wobble reduction. |
| **`M=X`** | Sets RMS Limit | Adjusts explicitly the electrical current mapped natively by the TMC2208 drivers. Limits heat saturation. *(Default 1100mA).* |
| **`S`** | Output View | Dumps configured memory buffers sequentially displaying active parameters directly back identically mapped through UART protocols. |
| **`C`** | Calibrate | Completely zeroes drifting Mahony values identically measuring unmoving ambient vibration levels. **(Perform Strictly Stationary)** |
