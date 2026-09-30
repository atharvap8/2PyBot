# BaseLink Code Reference: 126 Checks

References are relative to [firmware/BaseLink](../../firmware/BaseLink/). Runtime keys come from `params.h`; use `P,key,value` at 460800 baud. See [PROTOCOL.md](../../firmware/BaseLink/PROTOCOL.md) for record fields and commands.

## Balance and state

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 1 | Which controller balances the robot? | `BaseLink.ino::runController()` | LQR/LQI state feedback; no active pitch PID module |
| 2 | E is accepted but motors stay off | `loop()`, `P.armAngle` | Arming waits for corrected pitch below `ARMANG`, default 5 degrees |
| 3 | Robot falls immediately after arming | `pitchF`, `rateF`, motor direction | Verify sensor, encoder, and motor signs before adjusting gains |
| 4 | Normal hold drifts | `k1`, `k5`, `zInt` | Inspect geometry, trim, position error, integral limits, and saturation |
| 5 | Position recovery is too aggressive | `P.exClamp`, K1 | `EXCLAMP` bounds the position error used by hold |
| 6 | Slow integral recovery | K5, `P.zIntLim` | `ZLIM` bounds position integral; it is not a pitch integral |
| 7 | Why does the integral disappear during driving? | Driving branch | Velocity tracking resets `zInt` each loop |
| 8 | Braking parks away from the original point | `P.brakeLook`, `xRef` | Driving updates the hold reference from position and velocity |
| 9 | Stiff hold has no effect while driving | Controller branch order | Normal velocity tracking takes priority over stiff hold |
| 10 | Climb mode changes steering feel | `P.climbStr` | Climb scales steering authority, default 0.40 |
| 11 | Climb reference gets ahead of the robot | `xRef` clamp | Reference remains within `EXCLAMP` of measured position |
| 12 | Hold mode disappears after arming | `resetController()` | Arming resets stiff/climb flags; select the mode afterward |
| 13 | Speed saturates under a push | `satV`, `P.vMax` | Inspect motor authority, common velocity limit, and geometry |
| 14 | Acceleration saturates | `satA`, `P.aMax` | `MAXACC` determines the derived acceleration cap |
| 15 | Hold reference moves under saturation | Saturation-bleed block | Above 90% of `vMax`, `VSATBLD` moves reference toward position |
| 16 | R does not stop the robot | Serial `R`, `resetController()` | R resets controller state without disabling motors |
| 17 | Fall recovery does not arm automatically | `requestDisable()` | A new START press or E request is required |
| 18 | Where is STATE_FALLEN? | `RobotState` | Only idle and balancing states exist |
| 19 | Excessive tilt stops balancing | `P.maxTilt` | Default `MAXTILT` is 55 degrees |
| 20 | A rate spike stops balancing | `P.maxRate` | Default `MAXRATE` is 450 degrees/s |

## IMU and encoder estimation

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 21 | IMU begin fails | `IMUSensor::begin()` | Check SDA 21, SCL 22, power, address, and library |
| 22 | Sensor configuration fails | Enable/ODR/full-scale calls | Read the specific accelerometer or gyro error |
| 23 | Calibration has too few readings | `calibrateGyro()` | Check I2C; at least half the 500 samples must succeed |
| 24 | Calibration offsets vary | Gyro-bias averaging | Keep the sensor stationary for the sampling interval |
| 25 | Calibration failure does not halt setup | `setup()` | The sketch does not check `calibrateGyro()` return status |
| 26 | Pitch angle is noisy | `P.imuCutoff` | `IMUCUT` controls two cascaded pitch filters |
| 27 | Pitch response is delayed | `_filtAlpha` | Lower cutoff reduces noise but adds delay |
| 28 | Pitch-rate sign differs from pitch | `P.gyroSign`, `P.rateSign` | Check raw-axis and controller-frame signs separately |
| 29 | Sensor mounting axis is wrong | `PITCH_GYRO_AXIS`, accel axis macros | Axis selection remains compile-time |
| 30 | Mahony correction changes during motion | `currentKp` | Acceleration error above 0.2 g sets correction gain to 0.1 |
| 31 | Does compass data correct the quaternion? | Mahony block | Gravity correction uses accel/gyro; compass yaw is separate |
| 32 | Compass heading has an offset | `P.magOffX/Y/Z` | Inspect `MAGOFFX`, `MAGOFFY`, and `MAGOFFZ` |
| 33 | Compass axes are reversed | `P.magSignX/Y/Z` | Use the corresponding `MAGSGN` keys with ±1 |
| 34 | Compass init log appears with no sensor | `_compass.init()` | The log does not validate a successful detection |
| 35 | Does compass yaw steer the robot? | Heading block | Steering feedback is encoder differential |
| 36 | Angle remains unchanged after an I2C failure | `IMUSensor::update()` | Failed reads return early and retain previous measurements |
| 37 | O pitch differs from D pitchF | Odometry/debug blocks | O is raw filtered pitch; D applies TRIM and PITCHSGN |
| 38 | Left/right counts are zero | `setupPCNT()`, encoder pins | Check A/B wiring and both PCNT units |
| 39 | Right hardware count has opposite sign | `getPositionR()` | The getter negates right counts to match the left convention |
| 40 | Position scale is wrong | `params_recompute()` | Verify `WHEELR`, 4096 counts/revolution, and `COUNTSM` |

## Motors, drivers, and heading

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 41 | Right driver COMM ERROR | `setupDriver()`, Serial2 | Check RX 16, TX 17, PDN_UART, TX resistor, and address straps |
| 42 | Left driver COMM ERROR | `setupDriver()`, Serial1 | Check RX 18, TX 19 and the same UART conditions |
| 43 | What does OK verify? | Connection and IFCNT check | Connection result zero and one counted write; inspect microstep readback |
| 44 | Driver error does not trigger MAIN warning | `StepperControl::begin()` | begin currently returns true regardless of per-driver results |
| 45 | One motor remains disabled | `MOTOR_EN_PIN` | Both enables must share GPIO 32; no separate left enable exists |
| 46 | Motor direction is reversed | `setSpeeds()` | Review LEFT/RIGHT_DIR_INVERT in config.h |
| 47 | Loaded current differs from compiled current | `setupDriver()`, `P.motorMa` | NVS CURRENT is used during driver initialization |
| 48 | M change disappears after reload | `setCurrent()` | M writes hardware only; use P,CURRENT,value |
| 49 | Driver temperature is high | CURRENT, supply, mechanics | Compare loaded current with motor rating and cooling |
| 50 | Motor skips under correction | MAXACC, MAXSPD | Inspect acceleration demand, supply, current, and mechanical drag |
| 51 | UART settings change but wheels do not move | STEP/DIR ISR | UART configures drivers; motion requires step pulses and enable |
| 52 | Timer API compile error | Core-version conditional | Use the Bluepad32 package; timer code has 2.x and 3.x paths |
| 53 | Wheel request exceeds common speed cap | Steering split | Steering is added after common velocity limiting |
| 54 | Final rate exceeds timer capability | `setSpeeds()` | Each wheel is clamped to ±TIMER_FREQ_HZ |
| 55 | Encoder overflow causes a discontinuity | `pcnt_overflow_isr()` | Check overflow extension at ±30000 counts |
| 56 | Short encoder pulses disappear | PCNT glitch filter | Pulses shorter than 1.25 microseconds are filtered |
| 57 | Heading corrections spin the robot | `dErr`, `diffRateF` | Check encoder direction, YAWKP/YAWKD, and YAWMAX |
| 58 | Heading abruptly re-latches | YAWRELCH, YAWTILT | Large differential error or tilt captures a new heading |
| 59 | Steering is too abrupt | `P.steerSlew` | STRSLEW limits active-steering rate changes |
| 60 | A look command turns while stationary | `lookCounts` | A,value adds a yaw reference through track width and encoder scale |

## Gamepad and host input

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 61 | Gamepad will not connect | `btgamepad_begin()` | Use EVOFOX Home+B pairing mode and the Bluepad32 core |
| 62 | Pairing keys need clearing | BP32.forgetBluetoothKeys | Enable the existing call for one flash, then disable it |
| 63 | Buttons do nothing after connection | Center-calibration block | Both sticks must be near center before buttons are processed |
| 64 | START held while connecting does not arm | `_freshConnect` | First processed button levels are adopted, not treated as presses |
| 65 | SELECT must work independently of START | `_enableEvent` | SELECT emits an explicit stop event after connection/calibration |
| 66 | Which stick steers? | PAD_STEER_ON_LEFT_STICK | Default left X steers; right stick controls the camera |
| 67 | Steering is mirrored | `P.padStrSgn` | Set PADSTR to the appropriate ±1 sign |
| 68 | Forward is mirrored | `P.padFwdSgn` | Set PADFWD to the appropriate ±1 sign |
| 69 | Centered stick still drifts | Calibration and STICKDZ | Inspect captured offsets and active radial deadzone |
| 70 | PADDZ does not change response | `_shapeStick()` | Active mapping uses STICKDZ; the legacy _dz helper is unused |
| 71 | Centered stick produces invalid values | Radial rescaling | STICKDZ=0 permits division by zero at exact center |
| 72 | Steering snap produces invalid values | `_smoothstep()` | Keep SNAPOUT greater than SNAPIN |
| 73 | Steering is weak near straight travel | Snap taper | SNAPIN/SNAPOUT intentionally suppress lateral input near the forward axis |
| 74 | Input is gentle near center | `_expo()` | EXPOSTR and EXPODRV blend linear and cubic response |
| 75 | LB does not toggle stiff hold | Button mapping | LB toggles speed; D-pad UP toggles stiff hold |
| 76 | RB does not select HIGH speed | Button mapping | RB toggles the torch; LB toggles LOW/HIGH |
| 77 | Gamepad axes are ignored | Input arbitration | Fresh host V commands take priority, even with zero axes |
| 78 | V enable does not arm | `handleLine()` | Use START or E; only host 1-to-0 enable transitions request stopping |
| 79 | First V with en=0 does not stop balance | `radxaEnPrev` | There is no stop edge from the boot value 0; use X |
| 80 | Pad disconnect leaves balancing active | Disconnected input branch | Axes zero and timestamp expires; disconnect does not disarm |

## Parameters, serial, and auto-tuning

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 81 | No reply to a command | `pollSerial()` | Use 460800 baud and LF or CR termination |
| 82 | Long commands are dropped | `rxBuf[96]` | Keep command text within 95 characters |
| 83 | Old dollar-prefixed commands fail | `handleLine()` | Send current command forms without a dollar prefix |
| 84 | P= changes the camera | Legacy tuner branch | P= is pan; use P,K1,value for a gain |
| 85 | How are valid parameter keys found? | `params_dump()` | P? reports keys, bounds, groups, descriptions, and derived values |
| 86 | There are more than 63 parameters | PARAM_TABLE | Current table has 106 fields across ten groups |
| 87 | Parameter editor splits descriptions incorrectly | P row format | Split only the first six commas; descriptions may contain commas |
| 88 | Parameter write is rejected | `params_set()` | Read PE for unknown key or out-of-range value |
| 89 | Invalid text is accepted as zero | `atof()` parsing | The parser does not validate numeric syntax before conversion |
| 90 | A bounded pair is inconsistent | Independent bounds | Min/max and SNAPIN/SNAPOUT relationships are not checked |
| 91 | Compiled changes have no effect after flash | `params_load()` | Saved NVS values override defaults; inspect P? |
| 92 | Defaults should replace saved values | `params_defaults()` | PD changes RAM; PS then persists it |
| 93 | PL without NVS does not undo edits | `params_load()` | With no saved version, current RAM values are retained |
| 94 | Saved settings survive reboot | Preferences namespace | PS writes each field under 2pybot with version 1 |
| 95 | K1= accepts values outside bounds | Legacy gain branch | Direct assignment bypasses bounds; use the P command |
| 96 | Wobble routine rejects starting | `AT,wobble` | Robot must already be balancing |
| 97 | Wobble reduction has stopped | RMS/iteration/floor checks | Target is 0.45 degrees RMS, eight reductions, or 40% gain floor |
| 98 | Trim routine aborts when input moves | `at_update()` | Wobble/trim abort on input above 0.05 or loss of balancing |
| 99 | AT,done reports busy during radius run | `at_handleLine()` | Busy guard executes before radius completion; manual calibration is needed |
| 100 | X fails to abort stall search | Idle motor-control path | Use AT,stop; balance stop does not cancel stall auto-tuning |

## Terrain, driver tuning, display, and payload

| # | Question or symptom | Source | Check / behavior |
| :--- | :--- | :--- | :--- |
| 101 | Auto-tune result is lost after reboot | `at_finish()` | Results are RAM-only until PS |
| 102 | Auto-tune times out | AT_TIMEOUT_MS | All active routines time out after 45 seconds |
| 103 | Terrain adaptation should be disabled | `P.terrEnable` | P,TERREN,0 resets reported roughness and multiplier |
| 104 | Roughness responds too slowly | `P.terrTau` | TERRTAU controls the EMA time constant |
| 105 | Rough-ground control feels softer | `terrain_soften()` | Normal K3/K4 and climb C3/C4 are multiplied by the terrain factor |
| 106 | Stiff hold ignores terrain softening | Stiff branch | S3/S4 are not multiplied by the factor |
| 107 | Airborne state appears | DROPG, DROPMS | Magnitude below 0.55 g for 35 ms confirms it by default |
| 108 | Gains stay soft after touchdown | LANDMS | Landing holds full softening for 350 ms by default |
| 109 | Integral still changes while airborne | Saturation/driving branches | AIRFRZ suppresses accumulation, not bleed or driving reset |
| 110 | Terrain rate data trails measurement | Main loop ordering | terrain_update currently receives the previous rateF |
| 111 | Saved StallGuard settings differ at boot | `setupDriver()` | Boot uses compiled tuning; PL after initialization applies saved settings |
| 112 | Runtime CoolStep disable is needed | `applyTuning()` | Set CSSEMIN to zero |
| 113 | DIAG never reports | USE_DIAG_PINS | Wire GPIO 34/35 and enable the compile-time option |
| 114 | DIAG reports but does not stop motors | Main-loop DIAG block | Reporting is intentionally separate from balance stop conditions |
| 115 | Disarmed motors do not hold | `disable()` | Shared EN is HIGH; standstill current settings do not keep drivers enabled |
| 116 | Ring points in the wrong direction | LEDFRONT, LED_DIR_CW | Set front index within the ring; direction remains compile-time |
| 117 | Ring refresh or brightness is wrong | LEDFPS, LEDBRI | Runtime defaults are 50 Hz and 200/255 |
| 118 | Host owns control without a red ring | Hazard condition | Red requires balancing, zero drive input, and speed above 0.10 m/s |
| 119 | Pan/zoom wiring follows old comments | config.h pins | Actual pins are pan 26, zoom 0, torch 2 |
| 120 | Pan request exceeds physical output | `_servoWriteDeg()` | Physical pulse conversion clamps to 180 degrees |
| 121 | Servo stops pulsing while idle | `P.svIdleMs` | SVIDLE above zero releases pulses after inactivity |
| 122 | SVRATE does not change servo PWM | Payload refresh gate | It changes write cadence; PWM remains 50 Hz |
| 123 | Saved torch boot preset is ignored | `_torchPct` initializer | Actual preset uses TORCH_BOOT_PCT, not P.torchBoot |
| 124 | Torch boot message contains wrong numbers | `payload_begin()` | Float P.torchBoot is passed to an integer printf format |
| 125 | Debug records arrive faster than 10 Hz | DEBUG_PERIOD_MS | Value 10 means nominal 100 Hz; L toggles periodic debug |
| 126 | Radxa gate uses the wrong velocity scale | radxa_brain.py constants | Match firmware baud and loaded wheel radius before using O records |

## Command references

- [USB protocol and telemetry](../../firmware/BaseLink/PROTOCOL.md)
- [Configuration and tuning](Config_and_Tuning_Guide.md)
- [Common troubleshooting](Troubleshooting_Guide.md)
- [Program flow](Program_Flow_State_Machine.md)
