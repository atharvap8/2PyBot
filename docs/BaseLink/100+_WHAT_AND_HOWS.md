# BaseLink Firmware — Troubleshooting Reference
> 100+ situations mapped to source locations, constants, and functions.
> Format: **Symptom** → `file : function / constant` → Fix

---

## Part 1 — `config.h` Parameter Effects

| # | If you change / observe… | Affected constant | Affected function / location | Effect / fix |
|---|---|---|---|---|
| 1 | Robot leans forward at rest | `DEFAULT_TARGET_ANGLE` | `pid.setpoint` in `STATE_BALANCING` | Increase value slightly (e.g., 0.10) to shift balance point backward |
| 2 | Robot rocks violently left-right | `DEFAULT_KP` | `PIDController::compute()` | Kp is too high; reduce by 10–20% increments |
| 3 | Robot drifts slowly despite balancing | `DEFAULT_KI` | `PIDController::compute()` — integral term | Add small Ki (0.5–2.0) to eliminate steady-state offset |
| 4 | Robot reacts sluggishly to disturbances | `DEFAULT_KD` | `PIDController::compute()` — derivative term | Increase Kd; also check `PID_D_FILTER_ALPHA` is not too low |
| 5 | Motors hit max speed and stall | `PID_OUTPUT_MAX` / `PID_OUTPUT_MIN` | `PIDController::compute()` return clamp | Lower limit or reduce Kp so output stays within motor capability |
| 6 | Integral term explodes after a stall | `INTEGRAL_LIMIT` | `_integral` clamp in `compute()` | Reduce `INTEGRAL_LIMIT`; also call `pid.reset()` before re-enabling |
| 7 | Robot over-corrects only on large tilts | `ADAPTIVE_ERROR_THRESHOLD` | `effKp` in `compute()` | Threshold too low; raise to 6–8° so boost only fires on real falls |
| 8 | Adaptive boost causes oscillation | `ADAPTIVE_KP_BOOST` | `effKp = Kp * adaptiveBoost` | Reduce multiplier (e.g., 1.02); or disable with `useAdaptive = false` |
| 9 | Robot drives forward when commanded to stop | `MANUAL_DRIVE_RATE` | `targetAngleOffset` accumulation in `loop()` | Rate too high; reduce or check `driveState` is reset to 'X' |
| 10 | Robot tilts excessively during driving | `MAX_MANUAL_TILT` | `constrain(targetAngleOffset, ...)` in `loop()` | Lower ceiling; default 6.0° is usually appropriate |
| 11 | Robot brakes too sharply / abruptly | `BRAKE_DECAY_RATE` | EMA alpha `1 - exp(-BRAKE_DECAY_RATE * dt)` | Lower value (e.g., 4.0) for gentler deceleration |
| 12 | Drift correction oscillates position | `MAX_POS_HOLD_TILT` | `activeDriftCorrection` in drift block | Reduce to 1.0° or lower; too much authority causes hunting |
| 13 | Steering feels jerky / snappy | `STEER_SMOOTHING` | `steerAlpha` EMA in `loop()` | Lower value (e.g., 10.0) to ramp steering more gradually |
| 14 | IMU pitch is noisy / jittery | `IMU_FILTER_CUTOFF_HZ` | `_filtAlpha` in `IMUSensor::begin()` | Lower cutoff (e.g., 20 Hz) for smoother angle; adds phase lag |
| 15 | Robot stalls during fast corrections | `MOTOR_ACCEL_LIMIT` | `maxChange` in `STATE_BALANCING` rate limiter | Increase limit; too low prevents PID from reacting fast enough |
| 16 | Derivative term causes high-freq buzz | `PID_D_FILTER_ALPHA` | `_lastD` EMA in `compute()` | Lower alpha (e.g., 0.1–0.2) to damp derivative noise |
| 17 | Heading drifts during straight drive | `YAW_KP` / `YAW_KD` | `yawPid.computeAngle()` in `STATE_BALANCING` | Increase YAW_KP; ensure `enableYawPID = true` via `$EN_Y=1` |
| 18 | Yaw correction overshoots target | `MAX_STEERING` | `yawPid.outputMax` / `outputMin` | Reduce `MAX_STEERING`; also lower `YAW_KP` |
| 19 | Robot falls immediately at max tilt | `MAX_TILT_ANGLE` | `fabsf(angle) > MAX_TILT_ANGLE` in `STATE_BALANCING` | Value is 95° by default; do not lower below ~45° during testing |
| 20 | Robot takes too long to start | `STARTUP_SETTLE_MS` | `delay(STARTUP_SETTLE_MS)` in `setup()` | Reduce to 1000 ms for faster iteration; 3000 ms ensures clean bias |

---

## Part 2 — IMU & AHRS (`imu_sensor.cpp` / `imu_sensor.h`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 21 | Serial prints `[IMU] ERROR: begin() failed` | `I2C_SDA`, `I2C_SCL` | `IMUSensor::begin()` | Check SDA=21 SCL=22 wiring; verify 3.3V power to IMU |
| 22 | Sensor ID reads `0x00` | — | `_sensor.ReadID()` in `begin()` | Wrong I2C address or faulty pull-up resistors on SDA/SCL |
| 23 | `Enable_X failed` error on boot | `IMU_ODR_HZ`, `IMU_ACCEL_FS` | `begin()` accel block | Library version mismatch; confirm ISM6HG256XSensor library is installed |
| 24 | Angle is stuck at 0° forever | `_filterSeeded` | `update()` EMA seeding block | `update()` not being called; check `imu.update(dt)` in `loop()` |
| 25 | Pitch angle slowly drifts over minutes | `MAHONY_KI` | `eInt_x/y/z` in `update()` | Ki too low; increase to 0.01; also recalibrate gyro |
| 26 | Pitch angle converges slowly after power-on | `MAHONY_KP` | `currentKp` in `update()` | Increase MAHONY_KP (e.g., 5.0) for faster initial convergence |
| 27 | Angle spikes violently when robot moves | `MAHONY_KP` vibration rejection | `accelMagGs` check in `update()` | 0.2g threshold triggers 0.1x Kp; this is intentional — check mechanical vibration |
| 28 | Gyro calibration prints `too few good readings` | `GYRO_CAL_SAMPLES` | `calibrateGyro()` | I2C bus error; check wiring and reduce `GYRO_CAL_SAMPLES` |
| 29 | Robot balances sideways (wrong axis) | `PITCH_ACCEL_PRIMARY`, `PITCH_GYRO_AXIS` | `update()` — axis selection block | Remap axes in `config.h` to match physical mounting orientation |
| 30 | Pitch angle sign is inverted (falls wrong way) | `PITCH_ACCEL_SIGN`, `PITCH_GYRO_SIGN` | `_accelAngle` and `_pitchRate` in `update()` | Flip sign constant from -1 to 1 (or vice versa) |
| 31 | `getPitchRate()` returns wrong sign | `PITCH_GYRO_SIGN` | `_pitchRate` calculation in `update()` | Flip `PITCH_GYRO_SIGN`; does not affect balance angle directly |
| 32 | Heading (yaw) drifts with tilt | `MAG_SIGN_X/Y/Z` | Tilt-compensated yaw block in `update()` | Sign flags misaligned with magnetometer orientation; calibrate offsets |
| 33 | Yaw jumps ±180° erratically | — | `atan2f(Yh, Xh)` in `update()` | Normal ±180° boundary; `computeAngle()` handles wrapping in yaw PID |
| 34 | Accelerometer pitch disagrees with AHRS pitch | `MAHONY_KP` | `getAccelAngle()` vs `getPitch()` | Normal during motion; accel-only angle is noisy — use `getPitch()` for control |
| 35 | IMU shows correct angle but robot still falls | `DEFAULT_TARGET_ANGLE` | `pid.setpoint` | Physical center of gravity differs from 0°; tune `DEFAULT_TARGET_ANGLE` |
| 36 | `update()` silently returns early | I2C bus state | `Get_X_Axes()` / `Get_G_Axes()` check in `update()` | I2C error during run; check for long wire lengths or missing pull-ups |
| 37 | Filter adds too much lag (robot is slow to respond) | `IMU_FILTER_CUTOFF_HZ` | `_filtAlpha` in `begin()` | Raise cutoff frequency (e.g., 80 Hz); reduces lag at cost of more noise |
| 38 | Mahony quaternion becomes NaN | — | `recipNorm` division in `update()` | Zero accelerometer vector; check IMU power and I2C integrity |
| 39 | `calibrateGyro()` shows large offsets (>500 mdps) | — | `_gyroOffX/Y/Z` in `calibrateGyro()` | Robot was moving during calibration; repeat with robot stationary on flat surface |
| 40 | Compass never initializes (no `[IMU] QMC5883L` output) | `I2C_SDA`, `I2C_SCL` | `_compass.init()` in `begin()` | QMC5883L shares I2C bus; verify address 0x0D is free and wiring is correct |


## Part 3 — Motors & Steppers (`stepper_control.cpp` / `stepper_control.h`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 41 | `[STEP] RIGHT TMC2226: COMM ERROR (conn=.., IFCNT delta=..)` on boot | `RIGHT_UART_TX`, `RIGHT_UART_RX`, `DRV_UART_ADDR` | `setupDriver()`, `test_connection()` and IFCNT write check | Check the 1 kΩ inline resistor on TX and the PDN_UART routing; MS1/MS2 must be LOW (UART address 0); verify `Serial2` pins (16/17) |
| 42 | `[STEP] LEFT TMC2226: COMM ERROR (conn=.., IFCNT delta=..)` on boot | `LEFT_UART_TX`, `LEFT_UART_RX`, `DRV_UART_ADDR` | `setupDriver()`, `test_connection()` and IFCNT write check | Same as above for `Serial1` remapped to pins 18/19 |
| 43 | Motors do not energize after pressing 'E' | `RIGHT_EN_PIN`, `LEFT_EN_PIN` | `enable()`, `digitalWrite(EN, LOW)` | EN pins wired incorrectly; the TMC2226 enables on LOW, same as the TMC2208; verify GPIO 32 and 26 |
| 44 | Motors spin in wrong direction | `RIGHT_DIR_INVERT`, `LEFT_DIR_INVERT` | `setSpeeds()` — `pinDirL/R` XOR logic | Flip the invert flag for the offending motor in `config.h` |
| 45 | Motors stall under any load | `MOTOR_CURRENT_MA` | `setupDriver()`, `drv.rms_current()` | Raise `MOTOR_CURRENT_MA` (default 1500; silicon limit 2000 mA RMS, motor rating permitting); UART current control is verified working, Vref only matters if UART is down; rerun the 'z' stall test after any change |
| 46 | Motors run hot / driver overheats | `MOTOR_CURRENT_MA` | `setupDriver()`, `drv.rms_current()` | Reduce current; add a heatsink to the TMC2226; CoolStep already lowers current with load above `DRV_TCOOLTHRS`, and IHOLD halves it at true standstill when disarmed |
| 47 | One motor runs, the other does not | GPIO wiring | `begin()` — `pinMode()` and `digitalWrite()` | Confirm STEP/DIR/EN pins match `config.h` for each motor |
| 48 | Motor speed maxes out immediately | `TIMER_FREQ_HZ`, `PID_OUTPUT_MAX` | `setSpeeds()` clamp check | Output saturated; check PID gains or increase `PID_OUTPUT_MAX` |
| 49 | Motors jitter at low speeds | `TIMER_FREQ_HZ`, microstep distribution | `tick()` — Bresenham accumulator | Normal at very low steps/s; ensure `MOTOR_ACCEL_LIMIT` is not clamping small changes |
| 50 | Motor makes grinding noise during balance | `MICROSTEPS`, `DRV_STEALTHCHOP` | `setupDriver()`, `drv.microsteps()` | Resolution is programmed over UART at 1/8 with interpolation to 1/256 (`intpol`), so grinding is rarely a microstep problem; check mechanics first, then confirm `DRV_STEALTHCHOP 1`. Raising `MICROSTEPS` changes `STEPS_PER_M` and invalidates tuned gains |
| 51 | Robot spins in place when commanded forward | `LEFT_DIR_INVERT` / `RIGHT_DIR_INVERT` | `setSpeeds()` XOR | Both invert flags have the same value; one must differ from the other |
| 52 | `timerBegin()` crashes or returns null | ESP32 Arduino Core version | `begin()` — `timerBegin(1000000)` | Core 3.0+ API used; confirm Board Manager version ≥ 3.0 |
| 53 | Step pulses stop completely mid-run | ISR crash / stack overflow | `tick()` in IRAM | Confirm `tick()` has `IRAM_ATTR`; avoid heap calls inside ISR |
| 54 | `disable()` does not stop movement | Mutex not entered | `disable()` — `portENTER_CRITICAL_ISR` | `timerMux` must be unlocked; do not call `disable()` from inside ISR |
| 55 | `setCurrent()` has no visible effect | TMC2226 UART not responding | `setCurrent()`, `rms_current()` | UART must be working at runtime; confirm the boot log shows `TMC2226: OK` with a 1/8 readback and no COMM ERROR |
| 56 | `getPositionL()` always returns 0 | PCNT not initialized | `getPositionL()` — `pcnt_get_counter_value()` | Confirm `setupPCNT()` ran without error; check `ENC_LEFT_A/B` pin assignments |
| 57 | Encoder position counts backwards | `getPositionR()` negation | `getPositionR()` — `return -(...)` | Right encoder negated to match left; flip negation if encoders are swapped |
| 58 | Position wraps / resets unexpectedly | `PCNT_H_LIM` / `PCNT_L_LIM` | `pcnt_overflow_isr()` | Overflow ISR must be registered; confirm `pcnt_isr_service_install()` succeeded |
| 59 | Robot bounces off walls without stopping | `MAX_TILT_ANGLE` | Fall detection in `STATE_BALANCING` | Angle threshold is 95°; reduce if robot is physically constrained to smaller range |
| 60 | Stepper makes high-pitched whine | `TIMER_FREQ_HZ` | `timerAlarm()` in `begin()` | 20 kHz is at the threshold of hearing; increase to 25 kHz if hardware allows |


## Part 4 — PID Controller (`pid_controller.h`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 61 | Robot oscillates at a fixed frequency | `DEFAULT_KP` | `compute()` — `_lastP` | Classic Kp-induced oscillation; reduce Kp by 10–15% until stable |
| 62 | Robot oscillates and Kd makes it worse | `PID_D_FILTER_ALPHA` | `_lastD` EMA in `compute()` | Alpha too high (unfiltered); lower to 0.15–0.25 to suppress noise-driven D |
| 63 | Integral term causes slow drift after re-enable | `_integral` not reset | `reset()` | Always call `pid.reset()` before `steppers.enable()` |
| 64 | Robot jerks on setpoint change (joystick push) | Setpoint kick | `compute()` uses derivative-on-measurement | This is handled by design; if still jerky check `MOTOR_ACCEL_LIMIT` |
| 65 | `getP()` / `getI()` / `getD()` all return 0 | `_firstRun = true` | `compute()` first-run guard | Called before first `compute()` invocation; values populate after first call |
| 66 | Kp change via Serial has no effect | `pid.Kp` field | `SerialTuner::process()` case 'P' | Verify Serial monitor sends newline (`\n`) at end of command |
| 67 | Adaptive boost fires constantly | `ADAPTIVE_ERROR_THRESHOLD` | `effKp` check in `compute()` | Threshold too low relative to typical operating error; raise to 6–10° |
| 68 | Yaw PID corrects in wrong direction | `invertYaw` flag | `yawOutput = -yawOutput` in `STATE_BALANCING` | Toggle `invertYaw` via `$INV_Y=1` or `$INV_Y=0` over Bluetooth |
| 69 | Yaw PID enabled but heading still drifts | `YAW_KI` | `yawPid` integral in `computeAngle()` | Add small Ki (0.05–0.2); or check that `enableYawPID = true` |
| 70 | `computeAngle()` causes 360° spin on enable | `_firstRun` in yaw PID | `yawPid.reset()` call | Always call `yawPid.reset()` and re-capture `targetYaw = imu.getYaw()` before enabling |
| 71 | Output is always at max / min limit | `PID_OUTPUT_MAX` too low | `constrain()` in `compute()` | Loosen output bounds or reduce gains; robot cannot recover from saturation |
| 72 | I term accumulates during motor-disabled period | `_integral` not cleared | `reset()` not called | Call `pid.reset()` on every 'E' enable command (already done in `SerialTuner`) |
| 73 | D term causes chatter on flat surface | `PID_D_FILTER_ALPHA` | `_lastD` EMA | Robot is too sensitive to IMU noise; lower alpha to 0.1 |
| 74 | Balance PID fights drift correction | `MAX_POS_HOLD_TILT` vs `PID_OUTPUT_MAX` | `desired` fed into `targetAngleOffset` | Drift tilt limit (1.5°) must be well within the PID's controllable range |
| 75 | Serial command `R` does not stop oscillation | `reset()` clears integrator only | `PIDController::reset()` | Gains are not changed by reset; also adjust Kp/Kd via `P=` / `D=` commands |


## Part 5 — ESP-NOW & Bluetooth (`espnow_comm.h`, `BaseLink.ino`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 76 | `[ESPNOW] Init FAILED` on boot | WiFi radio conflict | `espnow_receiver_begin()` | Ensure `WiFi.mode(WIFI_STA)` is called before `esp_now_init()` |
| 77 | Joystick has no effect on robot | `JOY_TIMEOUT_MS` | `joyActive` check in `loop()` | Packet not arriving; confirm `receiverMAC[]` in Controller.ino matches robot STA MAC |
| 78 | Robot goes to zero speed 200 ms after joystick release | `JOY_TIMEOUT_MS = 200` | `joyActive` flag in `loop()` | Expected behavior; increase `JOY_TIMEOUT_MS` if timeout is too aggressive |
| 79 | Bluetooth `SerialBT` commands are ignored | `btBuffer` parsing | `while (SerialBT.available())` in `loop()` | Commands must be prefixed with `$` and terminated with `\n` |
| 80 | `$KP=1500` does not update Kp | Prefix parsing | `btBuffer.startsWith("$")` check | Ensure no space before `$`; check `eqIdx > 0` logic in Bluetooth parser |
| 81 | `$EN_P=1` does not enable drift correction | `enableDriftCorrection` | Bluetooth parser — `key == "EN_P"` | Check that the value sent is `1` not `1.0` (both work via `val > 0.5f`) |
| 82 | `$EN_Y=1` enables yaw PID but heading jumps | `targetYaw` not captured | `enableYawPID` block in Bluetooth parser | `targetYaw = imu.getYaw()` is called on enable edge; ensure IMU is active |
| 83 | Bluetooth connects but telemetry is garbled | `DEBUG` flag | `SerialBT` output | Bluetooth shares data with USB Serial; use one at a time for clean output |
| 84 | ESP-NOW receiver MAC is wrong | `receiverMAC[]` | Controller.ino `setup()` | Read STA MAC from `[ESPNOW] Receiver MAC` line in robot serial output |
| 85 | Joystick button enables but robot immediately falls | `pid.reset()` not called | Enable edge in `joyEnable` handler | `pid.reset()` is called on the enable edge; verify `angle < 5°` when enabling |
| 86 | `$X` command over Bluetooth does not stop motors | Bluetooth command parser | `key == "X"` without `=` sign | `$X` must be sent without `=`; confirm `eqIdx <= 0` branch is reached |
| 87 | W/A/S/D keyboard commands do not work | `joyActive` flag | `else` branch of joystick block | Commands only active when `joyActive = false` (joystick timed out); expected design |
| 88 | Steering is reversed (left goes right) | `joySteering` sign | `targetSteering = joySteering * TURN_SPEED_DEG_SEC` | Negate `joySteering` in the packet or flip in Controller.ino before sending |
| 89 | No `[ESPNOW] Receiver ready` output | `espnow_receiver_begin()` | `loop()` in `setup()` | Function called after `SerialBT.begin()`; move ESP-NOW init before Bluetooth |
| 90 | Joystick forward maps to backward motion | `joyForward` sign | `float desired = -joyFwd` in `loop()` | Negation is intentional (joystick push = forward lean = negative angle); flip if mechanically reversed |


## Part 6 — Encoder Odometry & Drift Correction (`stepper_control.cpp`, `BaseLink.ino`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 91 | `getAveragePosition()` always returns 0 | PCNT hardware not started | `setupPCNT()` in `begin()` | Check encoder wiring (A/B channels); confirm PCNT units 0 and 1 initialize |
| 92 | Position counter never overflows | `PCNT_H_LIM = 30000` | `pcnt_overflow_isr()` | ISR not registered; confirm `pcnt_isr_service_install()` succeeds with return check |
| 93 | Drift correction triggers while standing still | `DRIFT_ERROR_SETPOINT = 500` | Drift eval block in `loop()` | Threshold too low; encoder noise accumulates — raise to 800–1200 ticks |
| 94 | Drift correction does not engage during real drift | `DRIFT_EVAL_LOOPS` | Drift counter check in `loop()` | Evaluation window is 0.5 s (100 loops at 200 Hz); wait longer before judging |
| 95 | Drift correction oscillates back and forth | `MAX_POS_HOLD_TILT` | `activeDriftCorrection` assignment | Reduce tilt authority; value of 1.5° may be too strong on a compliant floor |
| 96 | Drift correction fights the joystick | `driving` flag | `if (!driving)` gate before drift block | Drift resets correctly when `joyFwd > 0.05`; check that reset lines are reached |
| 97 | `driftLoopCounter` never reaches `DRIFT_EVAL_LOOPS` | `DRIFT_EVAL_PERIOD_HZ = 2.0` | `DRIFT_EVAL_LOOPS` macro | At 200 Hz, `DRIFT_EVAL_LOOPS = 100`; verify `loop()` is running at correct rate |
| 98 | Left and right encoder counts disagree at rest | Mechanical encoder slip | `getPositionL()` vs `getPositionR()` | Check encoder magnet gap and alignment; electrical noise on A/B lines |
| 99 | Encoders count in opposite directions | `getPositionR()` inversion | `return -(encOverflowR + hw)` | Remove or add negation depending on physical wheel-to-encoder orientation |
| 100 | Glitch filter rejects valid encoder pulses | `pcnt_set_filter_value(unit, 100)` | `setupPCNT()` | 100 × 12.5 ns = 1.25 µs filter; lower to 50 if encoder generates shorter pulses |
| 101 | Position resets when robot is picked up | `_encOverflowL/R` not reset | No explicit reset function | Add a `resetPosition()` method that zeroes `_encOverflowL/R` and clears PCNT |
| 102 | Drift correction activates immediately on enable | `lastDriftEvalTicks` not set | Enable transition in `STATE_IDLE` block | `lastDriftEvalTicks = steppers.getAveragePosition()` is called on enable — verify it runs |
| 103 | Robot drifts faster over time | Integral wind-up + drift | `_integral` accumulation | Enable drift correction (`$EN_P=1`) and ensure `INTEGRAL_LIMIT` prevents accumulation |
| 104 | `activeDriftCorrection` is non-zero but robot ignores it | `enableDriftCorrection = false` | `if (enableDriftCorrection && ...)` | Send `$EN_P=1` via Bluetooth or set `enableDriftCorrection = true` in code |
| 105 | One encoder counts faster than the other | Unequal wheel diameter or slip | `getAveragePosition()` | Mechanical issue; also check that `ENCODER_PPR = 1024` matches actual encoder |


## Part 7 — State Machine, Serial Tuner & General Boot Issues (`BaseLink.ino`, `serial_tuner.h`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 106 | Robot stays in `STATE_IDLE` after pressing 'E' | `fabsf(angle) < 5.0f` guard | `STATE_IDLE` transition block in `loop()` | Robot is tilted more than 5° at enable time; hold upright before pressing 'E' |
| 107 | Robot enters `STATE_FALLEN` immediately on enable | `MAX_TILT_ANGLE` | Fall check in `STATE_BALANCING` | Angle already exceeds 95°; enable only when robot is near-vertical |
| 108 | Robot stuck in `STATE_FALLEN` — never recovers | `steppers.isEnabled()` check | `STATE_FALLEN` block in `loop()` | Re-enable motors via 'E' command; robot checks `fabsf(angle) < MAX_TILT_ANGLE` |
| 109 | Serial command 'E' enables but state stays IDLE | `angle < 5.0f` precondition | `STATE_IDLE` → `STATE_BALANCING` transition | The angle check runs in the next loop iteration; tilt robot upright before enabling |
| 110 | `[MAIN] FATAL: IMU init failed` on every boot | `imu.begin()` return | `setup()` — `while(true)` halt | Hardcoded halt; fix IMU wiring; check I2C address conflicts |
| 111 | Serial output appears twice (duplicated lines) | Dual output — USB + Bluetooth | `Serial.printf()` + `SerialBT` | `SerialBT.begin()` echoes to USB in some Arduino core versions; use one interface |
| 112 | `[TUNE] Unknown command` for every keypress | Command parsing | `SerialTuner::process()` | Serial monitor sends `\r\n`; ensure `line.trim()` strips carriage return |
| 113 | PID settings printed by 'S' differ from config.h | Live-tuned values | `printSettings()` in `serial_tuner.h` | Expected — runtime changes via `P=` / `I=` / `D=` override compile-time defaults |
| 114 | `[TUNE] Alpha setting deprecated` appears | Deprecated 'A' command | `case 'A'` in `SerialTuner::process()` | Mahony filter replaced the old alpha-based complementary filter; ignore this message |
| 115 | LED blinks fast even when standing still | `absAngle` in blink interval | `map(absAngle, 0, 15, 1000, 40)` | Fast blink = high tilt angle; robot is not balanced; check `DEFAULT_TARGET_ANGLE` |
| 116 | LED never blinks in IDLE state | `lastBlinkMs` comparison | `STATE_IDLE` LED block | 500 ms blink period; ensure `millis()` is running (not blocked by `delay()` calls) |
| 117 | `loopCounter` overflows | `unsigned long` (32-bit) | `loopCounter++` in `loop()` | Overflows after ~248 days at 200 Hz; use `loopCounter % X == 0` safely |
| 118 | Main loop runs slower than 200 Hz | `LOOP_PERIOD_US` | `if (elapsedUs < LOOP_PERIOD_US) return` | Check for blocking `delay()` calls added in custom code; keep ISR overhead low |
| 119 | Telemetry output floods Serial at high rate | `LOOP_FREQ_HZ / 5` | Debug print in `loop()` | Output is throttled to 5 Hz; increase divisor (e.g., `/10`) for 2 Hz if needed |
| 120 | `DEBUG = false` still prints messages | `DEBUG` flag scope | `if (...DEBUG)` telemetry block | Only the periodic telemetry block is gated; `[MAIN]` state messages always print |

## Part 8: TMC2226 StallGuard4, CoolStep and Chopper (`stepper_control.cpp`, `config.h`)

| # | Symptom | Constant / variable | Function | Fix |
|---|---|---|---|---|
| 121 | Boot log shows `SGTHRS=0` or StallGuard values look dead | `DRV_STEALTHCHOP` | `setupDriver()` | StallGuard4 and CoolStep only work in StealthChop; set `DRV_STEALTHCHOP 1` |
| 122 | `[STALL]` lines print during normal driving | `SGTHRS_LEFT` / `SGTHRS_RIGHT` | DIAG check in `loop()` | Thresholds too sensitive; lower them (stall flags when SG_RESULT < 2 x SGTHRS). Tuned values on this hardware are 76 and 77 |
| 123 | Real stalls never raise a flag | `SGTHRS_LEFT` / `SGTHRS_RIGHT`, `DRV_TCOOLTHRS` | DIAG check in `loop()` | Raise the thresholds, and remember flags are only valid above `DRV_TCOOLTHRS` (about 1250 microsteps per second); slow stalls are invisible to StallGuard |
| 124 | No `[STALL]` output at all | `USE_DIAG_PINS` | `begin()` DIAG interrupt setup | Reporting is off by default; wire DIAG to GPIO 34 (left) and GPIO 35 (right) and set `USE_DIAG_PINS 1`. Output is report-only and never disables the motors |
| 125 | Balance feels soft against pushes while driving | `COOLSTEP_ENABLE` | `setupDriver()` CoolStep block | CoolStep can drop current to half of IRUN under light load; set `COOLSTEP_ENABLE 0`. Standstill balance is unaffected because CoolStep is inactive below `DRV_TCOOLTHRS` |
| 126 | Holding torque weak when disarmed | `drv.ihold(16)` | `setupDriver()` | IHOLD is set to roughly half current at true standstill; raise the value if the robot must hold position on a slope while disarmed |

---

## Quick Reference — Serial Commands

| Command | Action | Location |
|---|---|---|
| `E` | Enable motors + reset PID | `SerialTuner::process()` case 'E' |
| `X` | Emergency stop | `SerialTuner::process()` case 'X' |
| `C` | Recalibrate gyro bias | `SerialTuner::process()` case 'C' |
| `R` | Clear PID state | `SerialTuner::process()` case 'R' |
| `S` | Print current gains | `SerialTuner::printSettings()` |
| `P=<v>` | Set Kp live | `pid.Kp = val` |
| `I=<v>` | Set Ki live | `pid.Ki = val` |
| `D=<v>` | Set Kd live | `pid.Kd = val` |
| `T=<v>` | Set balance setpoint (°) | `pid.setpoint = val` |
| `M=<v>` | Set motor current (mA) | `steppers.setCurrent()` |
| `L` | Toggle debug telemetry | `DEBUG = !DEBUG` |

## Quick Reference — Bluetooth Commands (prefix `$`, suffix `\\n`)

| Command | Effect |
|---|---|
| `$KP=1250` | Set balance Kp |
| `$KI=0` | Set balance Ki |
| `$KD=1.786` | Set balance Kd |
| `$T=0.1` | Set target angle |
| `$EN_P=1` / `$EN_P=0` | Enable / disable drift correction |
| `$EN_Y=1` / `$EN_Y=0` | Enable / disable yaw PID |
| `$INV_Y=1` / `$INV_Y=0` | Invert yaw direction |
| `$YKP=20` | Set yaw Kp |
| `$YKD=5` | Set yaw Kd |
| `$PMT=1.5` | Set max drift correction tilt |
| `$E` | Enable motors |
| `$X` | Emergency stop |
| `$L` | Toggle debug output |


