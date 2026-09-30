# Program Flow and State Machine

Reference: [BaseLink.ino](../../firmware/BaseLink/BaseLink.ino). The balance state enum contains only `STATE_IDLE` and `STATE_BALANCING`.

## Boot sequence

1. Start USB serial at 460800 baud and wait 300 ms.
2. Load runtime parameters from NVS, or retain compiled defaults when no saved version exists. Apply `BOOTHIGH` to gamepad speed mode.
3. Initialize the LED ring and play its blocking startup animation.
4. Initialize I2C, IMU, and compass. Halt setup if IMU initialization returns false.
5. Disable the shared motor enable line, configure both TMC2226 drivers, initialize PCNT, and start the step timer.
6. Wait `STARTUP_SETTLE_MS`, default 3000 ms.
7. Average `GYRO_CAL_SAMPLES`, default 500, with 2 ms spacing. The sketch calls calibration but does not act on its return value.
8. Start Bluepad32 and initialize camera servos and torch.
9. Set the loop timestamp. Remain in `STATE_IDLE` until arming is requested.

Driver UART errors print `COMM ERROR`; `StepperControl::begin()` still returns true. The sketch's UART warning branch does not receive these individual failures.

## Loop order

The loop runs when at least 5000 microseconds have elapsed. `dt` uses that elapsed interval, capped at 0.05 s.

1. Read IMU data and calculate corrected pitch, terrain state, and pitch rate. Terrain currently receives the previous loop's `rateF`.
2. Read wheel counts, calculate position and encoder difference, and update filtered rates.
3. Poll USB commands and Bluepad32 input.
4. Consume gesture/hold-mode buttons and payload input. Update camera and torch independently of balance state.
5. Report optional DIAG changes and speed-mode LED events.
6. Consume START/SELECT events.
7. Select drive authority: fresh host, fresh gamepad, or zero.
8. Advance gesture playback; sufficiently large operator input cancels it.
9. Evaluate balance-state transitions and run the controller when balancing.
10. Advance auto-tuning.
11. Update the LED ring.
12. Emit odometry and optional debug records at their scheduled intervals.

## State transitions

| Current state | Condition | Action / next state |
| :--- | :--- | :--- |
| Idle | START or `E` | Set `motorsRequested`; stay idle until upright |
| Idle | Requested and `abs(pitchF) < P.armAngle` | Reset controller, enable motors, enter balancing |
| Balancing | `X`, SELECT, or host enable 1-to-0 | Clear request, synchronize gamepad armed state, disable motors, enter idle |
| Balancing | `abs(pitchF) > P.maxTilt` | Fall LED event, disable, enter idle |
| Balancing | `abs(rateF) > P.maxRate` | Fall LED event, disable, enter idle |
| Balancing | No stop condition | Run LQR/LQI and steering |

Default thresholds are 5 degrees to arm, 55 degrees for fall cutoff, and 450 degrees/s for the rate cutoff. After an automatic stop, a new START press or `E` request is needed.

`V` messages never request arming. A first host `en=0` does not stop balancing unless the previous host enable value was 1. Gamepad disconnect zeros its axes; it does not cause a balance-state transition.

## Controller reset

`resetController()` clears `vCmd`, `zInt`, and `uOut`; captures current position and encoder difference as references; and clears gestures, stiff hold, climb mode, and look input.

It runs on entry to balancing and on serial `R`. `R` does not turn motors off and does not restore parameter defaults.

## Auto-tune state is separate

`autotune.h` has idle, wobble, trim, radius, and stall modes. These are not balance states. Wobble and trim require balancing; stall search requires idle and can enable the motors itself.

`requestDisable()` does not cancel the auto-tune state. During stall search, use the implemented `AT,stop` path. Radius completion is currently blocked by the auto-tune busy check. See [configuration and tuning](Config_and_Tuning_Guide.md).

## Payload and display

Serial polling, sensors, camera controls, ring updates, and telemetry continue in idle. The torch starts off and is not switched off by `requestDisable()` in the current sketch. GPIO 2 is the torch pin, not an onboard blink indicator.
