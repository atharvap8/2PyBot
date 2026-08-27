/*
 * ============================================================
 *  config.h — 2PyBot BaseLink (Package B: LQR / LQI)
 * ============================================================
 *  Pins, hardware parameters and control gains in one place.
 *
 *  WHAT CHANGED vs the old firmware
 *  --------------------------------
 *  The old balance law mapped angle error DIRECTLY to wheel
 *  velocity. For a velocity-actuated (stepper) pendulum that
 *  system always keeps one unstable pole (~ +1.6 rad/s) — it
 *  can never stand still, which is exactly the drift you saw.
 *
 *  New structure (proven in simulation, see REPORT.md):
 *      LQR full-state feedback with position-integral augmentation
 *      u = -( K1*(x-xRef) + K2*v + K3*pitch + K4*pitchRate + K5*z )
 *      z = Integral(x - xRef);  u = wheel acceleration, integrated
 *      into the ISR velocity command.
 *  The gains below were computed offline by solving the continuous
 *  algebraic Riccati equation for the linearized velocity-driven
 *  pendulum (tools/simulate_controllers.py regenerates them). The
 *  integral state absorbs the unknown CoG offset -> zero drift.
 * ============================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  PIN DEFINITIONS — ESP32 38-Pin DevKit  (unchanged)
// ============================================================

// ---- Right Stepper Motor (TMC2208) ----
#define RIGHT_STEP_PIN    33
#define RIGHT_DIR_PIN     25
#define RIGHT_EN_PIN      32
#define RIGHT_UART_TX     17    // 1 kOhm inline resistor on TX line.
#define RIGHT_UART_RX     16

// ---- Left Stepper Motor (TMC2208) ----
#define LEFT_STEP_PIN     27
#define LEFT_DIR_PIN      14
#define LEFT_EN_PIN       26
#define LEFT_UART_TX      19    // 1 kOhm inline resistor on TX line.
#define LEFT_UART_RX      18

// ---- I2C Bus ----
#define I2C_SDA           21
#define I2C_SCL           22
#define I2C_CLOCK_HZ      400000
#define ONBOARD_LED       2

// ---- MT6816 ABZ Quadrature Encoders (PCNT hardware decode) ----
#define ENC_LEFT_A         4
#define ENC_LEFT_B         5
#define ENC_RIGHT_A       13
#define ENC_RIGHT_B       23
#define ENCODER_PPR       1024
#define ENCODER_CPR       (ENCODER_PPR * 4)   // 4096 counts/rev
#define PCNT_H_LIM        30000
#define PCNT_L_LIM       -30000

// ============================================================
//  MOTOR / DRIVER PARAMETERS  (unchanged)
// ============================================================
// TMC UART reports COMM ERROR on this robot, so the config value was
// never reaching the drivers -- they run at the MS1/MS2 pin strapping.
// Encoder data (18 Aug log) shows the real motion is 2x the 1/16
// assumption => the drivers are strapped to 1/8. Set to match reality.
// If you FIX the UART wiring, the firmware will program 1/8 too, so
// this stays consistent either way. (Verify after any change: while
// moving gently, debug 'v' must equal vCmd/STEPS_PER_M.)
#define MICROSTEPS           8
#define STEPS_PER_REV        200
#define USTEPS_PER_REV       (STEPS_PER_REV * MICROSTEPS)   // 3200
#define MOTOR_CURRENT_MA     1100
#define R_SENSE              0.11f
#define RIGHT_DIR_INVERT     false
#define LEFT_DIR_INVERT      true

// ============================================================
//  STEPPER TIMER  (unchanged — 20 kHz Bresenham ISR)
// ============================================================
#define TIMER_FREQ_HZ        20000
#define TIMER_PRESCALER       80
#define TIMER_ALARM_COUNT    (1000000 / TIMER_FREQ_HZ)

// ============================================================
//  GEOMETRY — measure these on YOUR robot!
// ============================================================
#define WHEEL_RADIUS_M       0.0350f   // measured on this robot (70 mm wheels)
#define STEPS_PER_M          ((float)USTEPS_PER_REV / (2.0f * PI * WHEEL_RADIUS_M))
#define COUNTS_PER_M         ((float)ENCODER_CPR   / (2.0f * PI * WHEEL_RADIUS_M))

// ============================================================
//  SIGN CONVENTIONS — set once, then forget
// ============================================================
// Internal control math uses a "forward-positive" frame:
// leaning forward = positive pitch, driving forward = positive speed.
//
// PITCH_FWD_SIGN: with the old IMU mapping (config unchanged below),
// leaning the robot FORWARD makes imu.getPitch() go NEGATIVE, so -1.
// Verify: enable debug ('L'), tilt robot forward, "pitchF" must go +.
#define PITCH_FWD_SIGN       (-1.0f)
//
// ENC_FWD_SIGN: push the robot FORWARD by hand (motors off) and watch
// the O-stream — encL/encR must INCREASE. If they decrease, set -1.
#define ENC_FWD_SIGN         (1.0f)
//
// RATE_FWD_SIGN: sign of imu.getPitchRate() alone. The gyro rate can be
// inverted relative to the pitch angle (the old firmware never used the
// gyro rate, so this was never tested on this hardware!). If inverted,
// the damping term becomes ANTI-damping -> robot whips itself over in
// under half a second even though Phases 1 and 2 pass.
// Test: while balancing send K4=+6.0 (LQR) / D=-0.07 (PID). If the robot
// suddenly balances, flip this sign, restore the gain, reflash.
// Verify after fix: debug 'w' must be POSITIVE while tilting forward.
#define RATE_FWD_SIGN        (+1.0f)   // CONFIRMED on hardware 19 Aug (w + when tilting fwd)
//
// Static pitch trim (deg, in imu.getPitch() units). Rough value is
// fine — the velocity-loop integral absorbs the remainder online.
#define PITCH_TRIM_DEG       0.071f

// ============================================================
//  BALANCE CONTROL — Package B: LQR (LQI)
// ============================================================
// Designed for: L_eff (CoM height) = 0.15 m, wheel R = 42.5 mm.
// Simulation shows the design tolerates >= +/-30 % error in L_eff
// with the SAME gains, so a rough estimate is fine. To redesign,
// edit L_EFF/Q/R in tools/simulate_controllers.py and rerun it.
//
// States (SI units, forward-positive):
//   x  position (m)      v  velocity (m/s)
//   th pitch (rad)       om pitch rate (rad/s)
//   z  Integral(x - xRef) (m*s)   <- kills steady-state drift/bias
// Input: u = wheel linear acceleration (m/s^2)
//
// Riccati solution (Q = diag(20, 6, 90, 1.5, 6), R = 0.30):
#define LQR_K1   (-12.7422f)   // per m       (x - xRef)
#define LQR_K2   (-10.6993f)   // per m/s     (v)
#define LQR_K3   (-48.2848f)   // per rad     (pitch)
#define LQR_K4   ( -6.0051f)   // per rad/s   (pitch rate)
#define LQR_K5   ( -4.4721f)   // per m*s     (position integral)
// Closed-loop poles: -17.55, -7.75, -1.73 +/- 0.87j, -0.58  (all LHP)
// Drive mode uses the K2..K4 subset — verified poles: -16.1, -7.2, -6.1
#define Z_INT_LIM        0.5f   // integral anti-windup clamp (m*s)
// BALANCE > POSITION safeguards:
#define EX_CLAMP_M       0.15f  // max position error fed to the LQR (m)
#define VSAT_BLEED       3.0f   // hold-point bleed rate when vCmd saturates (1/s)

// POSITION reference shaping
#define V_HOLD_MAX_MS        0.35f    // unused by LQR core, kept for parity
#define BRAKE_LOOKAHEAD_S    0.30f    // hold-point lead while driving (s)

// ============================================================
//  EXPRESSION / GAMEPAD FEATURES  (serial: G, H, A commands)
// ============================================================
// STIFF HOLD ("H,1" / D-pad UP): second LQR gain set, solved offline
// with heavy position weights Q=diag(600,40,90,1.5,200), R=0.30
// (same CARE / plant as tools/simulate_controllers.py; verified to
// reproduce the previous sets exactly before re-weighting).
// Poles -19.96, -7.74, -4.03 +/- 1.57j, -0.58 (all LHP, well damped).
// ~1.7x the position+integral authority of the old stiff set (~4.7x
// normal hold): fights pushes noticeably harder and rails the accel
// clamp at ~2.3 cm displacement instead of ~3.9 cm. Previous set for
// fallback: -35.5636 -21.1461 -65.5197 -8.1273 -14.1421.
// Auto-suspended while driving; cleared on disable.
#define LQR_S1               (-59.9721f)
#define LQR_S2               (-30.9190f)
#define LQR_S3               (-81.4373f)
#define LQR_S4               (-10.0906f)
#define LQR_S5               (-25.8199f)
//
// CLIFF / CLIMB MODE ("H,2" / D-pad DOWN): slope gain set + a control-
// structure change. Driving becomes a slow position-REFERENCE ramp
// tracked by the FULL 5-state LQI at all times (normal driving drops
// the integral, which is exactly what breaks on a slope). On an incline
// the equilibrium pitch shifts by asin((r/L)*sin(slope)) — ~4.6 deg at
// 20 deg — and cancelling it needs more integral state than Z_INT_LIM
// allows, so climb mode gets its own wider clamp.
// Weights Q=diag(50,30,160,3,20), R=0.30 — same CARE / plant as the
// other sets (model verified to reproduce them before re-weighting).
// Poles all real, all LHP: -24.7, -7.3, -3.4, -1.1, -0.8. Simulated
// 10/20/25 deg climbs track to mm-level with pitch at the geometric
// equilibrium.
// REALITY CHECK: raises the ceiling on GRIPPY RAMPS toward ~20-25 deg
// (traction- and motor-current-limited). It does NOT make 45 deg
// possible: straight up needs mu >= tan(45) = 1.0 at the tire, and
// SIDEWAYS is strictly worse — static rollover at atan(halfTrack/L)
// ~ 27 deg with ZERO roll control. Work up 10 -> 15 -> 20 deg.
// DOWNHILL: fully supported, same branch — the math is symmetric.
// Simulated 20 deg descents (nose-first and backing down) track to
// mm-level just like climbs; steppers brake with the same velocity
// lock they climb with.
// TRANSITIONS (crests/dips) are the real danger, and they are
// ACCELERATION-AUTHORITY limited, not gain limited: mid-transition
// the stale integral still holds the OLD slope's torque while the
// NEW slope demands its own — at +/-20 deg that sums to ~1.57 m/s^2
// vs the ~1.37 m/s^2 available (MOTOR_ACCEL_LIMIT), the command
// rails, vCmd hits V_MAX, and the robot falls at the crest.
// Simulated envelope: +/-17 deg transitions clear at full climb
// speed (0.15 m/s); +/-20 deg clears only at ~0.08 m/s. Speed LOW
// (LB) scales the ramp to 0.45*0.15 = ~0.07 m/s — so the drill is:
// TAP LB BEFORE EVERY CREST OR DIP. Raising MOTOR_ACCEL_LIMIT
// (after fixing driver current) widens this envelope directly.
// Toggle OFF on flat ground (the wound integral re-clamps on exit).
#define LQR_C1               (-21.5836f)
#define LQR_C2               (-18.3212f)
#define LQR_C3               (-66.5207f)
#define LQR_C4               ( -8.3370f)
#define LQR_C5               ( -8.1650f)
#define CLIMB_Z_INT_LIM      2.0f     // climb integral clamp (m*s)
#define CLIMB_VEL_MS         0.25f    // full-stick reference ramp (m/s)
#define CLIMB_STEER_SCALE    0.40f    // steering authority while climbing
//
// LOOK ("A,<-1..1>"): sustained head-turn (yaw offset) while
// station keeping. Physically sustainable, unlike a pitch lean.
#define TRACK_WIDTH_M        0.155f   // wheel-to-wheel distance (m) -- MEASURE THIS
#define LOOK_MAX_DEG         18.0f    // full stick = this much turn
//
// GESTURES ("G,yes" "G,no" "G,spin" "G,dance" "G,stop"): scripted
// keyframes fed into the SAME drive inputs as the joystick, so
// balancing and every safety limit stay in charge throughout.

// ============================================================
//  SPEED / ACCELERATION LIMITS
// ============================================================
// Your motors STALL above ~9000 pulses/s (encoders proved it: v -> 0
// while vCmd railed). Cap well below the stall point; raise only after
// fixing driver current (Vref / UART).
#define MAX_SPEED_STEPS      7300.0f           // ~0.82 m/s at 1/8 ustep
#define MOTOR_ACCEL_LIMIT    20000.0f          // steps/s^2 (~2.75 m/s^2, stall-safe)
#define V_MAX_MS             (MAX_SPEED_STEPS   / STEPS_PER_M)
#define A_MAX_MS2            (MOTOR_ACCEL_LIMIT / STEPS_PER_M)

// ============================================================
//  DRIVE INPUT SCALING (Radxa 'V' line and ESP-NOW joystick)
// ============================================================
#define MAX_DRIVE_VEL_MS     0.70f    // leaves balance headroom under V_MAX (0.82)
#define DRIVE_DEADBAND       0.03f    // |forward| below this = position hold
// ESP-NOW packet carries "pitch offset" -5..+5 deg (old controller fw);
// convert to normalized forward command (-1..+1), forward-positive:
#define JOY_FWD_SCALE        (-0.2f)
#define JOY_STEER_SCALE      (1.0f)

// ============================================================
//  YAW / STEERING (encoder-differential heading hold)
// ============================================================
#define MAX_STEER_STEPS      6000.0f  // max differential half-split (steps/s)
#define STEER_SLEW           15000.0f // steering slew rate (steps/s^2)
#define STEER_DEADBAND       0.05f
#define YAW_HOLD_KP          1.5f     // steps/s per count of diff error
#define YAW_HOLD_KD          0.02f    // steps/s per (count/s) of diff rate
#define YAW_HOLD_MAX_STEPS   1200.0f  // heading-hold authority cap (steps/s)
// Wheel slip during a hard recovery shifts the L/R encoder diff by
// thousands of counts; "unwinding" that spins the robot in circles
// (~17.5k counts = one full turn on this geometry). Error beyond ~1/8
// turn is treated as slip and the current heading is re-latched.
#define YAW_RELATCH_COUNTS   2000.0f
#define YAW_TILT_SUSPEND_DEG 12.0f    // no heading corrections while tilted

// ============================================================
//  LINK TIMEOUTS / SAFETY
// ============================================================
#define RADXA_TIMEOUT_MS     600      // 'V' line freshness window
#define JOY_TIMEOUT_MS_CFG   200      // ESP-NOW freshness window
#define ARM_ANGLE_DEG        5.0f     // must be this upright to start
#define MAX_TILT_ANGLE       55.0f    // fall cutoff (deg)
#define MAX_PITCH_RATE_SAFETY 450.0f  // spike cutoff (deg/s)
#define STARTUP_SETTLE_MS    3000

// ============================================================
//  IMU OUTPUT FILTER  (unchanged)
// ============================================================
#define IMU_FILTER_CUTOFF_HZ  50.0f

// ============================================================
//  AHRS / MAHONY FILTER  (unchanged)
// ============================================================
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.005f

// ============================================================
//  MAGNETOMETER (QMC5883L)  (unchanged)
// ============================================================
#define MAG_OFFSET_X  0.0f
#define MAG_OFFSET_Y  0.0f
#define MAG_OFFSET_Z  0.0f
#define MAG_SIGN_X  1
#define MAG_SIGN_Y  1
#define MAG_SIGN_Z  1
#define GYRO_CAL_SAMPLES      500

// ============================================================
//  IMU AXIS MAPPING  (unchanged)
// ============================================================
#define PITCH_ACCEL_PRIMARY    'Y'
#define PITCH_ACCEL_SECONDARY  'Z'
#define PITCH_ACCEL_SIGN        -1
#define PITCH_GYRO_AXIS        'X'
#define PITCH_GYRO_SIGN         -1

// ============================================================
//  IMU SENSOR SETTINGS  (unchanged)
// ============================================================
#define IMU_ODR_HZ             960.0f
#define IMU_ACCEL_FS           4
#define IMU_GYRO_FS            2000

// ============================================================
//  CONTROL LOOP TIMING / SERIAL
// ============================================================
#define LOOP_FREQ_HZ          200
#define LOOP_PERIOD_US        (1000000UL / LOOP_FREQ_HZ)
#define SERIAL_BAUD           115200
#define ODOM_PERIOD_MS        20      // 50 Hz O-stream to the Radxa
#define DEBUG_PERIOD_MS       100     // 10 Hz human-readable debug ('L')

// ============================================================
//  LED RING — WS2812 16-LED (NeoPixelBus, RMT hardware — never
//  disables interrupts, so the 20 kHz stepper ISR is untouched)
// ============================================================
#define LED_RING_PIN      15
#define LED_RING_COUNT    16
#define LED_FRONT_INDEX    0     // which LED physically faces FORWARD
#define LED_DIR_CW         1     // 1 if indices go clockwise seen from above, else 0
#define LED_MAX_BRIGHT    255     // 0..255 power cap (16 LEDs full white = ~1 A!)
#define LED_FPS           50

// ============================================================
//  BLUETOOTH GAMEPAD (EVOFOX One S via Bluepad32)
// ============================================================
// Emits joyForward in the same -5..+5 "pitch offset" units the old
// ESP-NOW controller used, so JOY_FWD_SCALE and the arbitration in
// the .ino are byte-for-byte unchanged.
#define PAD_FWD_SIGN     (+1.0f)  // flip if stick-up drives backward
#define PAD_STEER_SIGN   (+1.0f)  // flip if turning is mirrored
#define PAD_DEADZONE      0.08f

// ---- Dual speed mode: LB = LOW, RB = HIGH (pad sticks only) ----
// Scales the pad's drive/steer authority INSIDE the existing caps:
// MAX_DRIVE_VEL_MS and MAX_STEER_STEPS stay the hard ceilings, and the
// stall/balance caps (MAX_SPEED_STEPS, V_MAX_MS, A_MAX_MS2) are never
// touched — the balancer keeps full recovery authority in BOTH modes.
// Radxa vision override and gesture keyframes stay full-scale too.
// Keep both scales <= 1.0 (HIGH mode is defined as 1.0 = current feel).
#define SPEED_LO_DRIVE_SCALE  0.50f   // LOW: 0.45 * 0.40 = 0.18 m/s full stick
#define SPEED_LO_STEER_SCALE  0.50f   // LOW: 0.50 * 3000 = 1500 steps/s full stick
#define SPEED_BOOT_HIGH       0       // 0 = boot in LOW (safe), 1 = boot in HIGH

#endif // CONFIG_H
