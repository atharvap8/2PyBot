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
#define RATE_FWD_SIGN        (+1.0f)   // start same as PITCH_FWD_SIGN
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

// POSITION reference shaping
#define V_HOLD_MAX_MS        0.35f    // unused by LQR core, kept for parity
#define BRAKE_LOOKAHEAD_S    0.30f    // hold-point lead while driving (s)

// ============================================================
//  SPEED / ACCELERATION LIMITS
// ============================================================
// Your motors STALL above ~9000 pulses/s (encoders proved it: v -> 0
// while vCmd railed). Cap well below the stall point; raise only after
// fixing driver current (Vref / UART).
#define MAX_SPEED_STEPS      6000.0f           // ~0.82 m/s at 1/8 ustep
#define MOTOR_ACCEL_LIMIT    20000.0f          // steps/s^2 (~2.75 m/s^2, stall-safe)
#define V_MAX_MS             (MAX_SPEED_STEPS   / STEPS_PER_M)
#define A_MAX_MS2            (MOTOR_ACCEL_LIMIT / STEPS_PER_M)

// ============================================================
//  DRIVE INPUT SCALING (Radxa 'V' line and ESP-NOW joystick)
// ============================================================
#define MAX_DRIVE_VEL_MS     0.80f    // forward = +/-1.0 maps to this speed
#define DRIVE_DEADBAND       0.03f    // |forward| below this = position hold
// ESP-NOW packet carries "pitch offset" -5..+5 deg (old controller fw);
// convert to normalized forward command (-1..+1), forward-positive:
#define JOY_FWD_SCALE        (-0.2f)
#define JOY_STEER_SCALE      (1.0f)

// ============================================================
//  YAW / STEERING (encoder-differential heading hold)
// ============================================================
#define MAX_STEER_STEPS      3000.0f  // max differential half-split (steps/s)
#define STEER_SLEW           15000.0f // steering slew rate (steps/s^2)
#define STEER_DEADBAND       0.05f
#define YAW_HOLD_KP          1.5f     // steps/s per count of diff error
#define YAW_HOLD_KD          0.02f    // steps/s per (count/s) of diff rate

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

#endif // CONFIG_H
