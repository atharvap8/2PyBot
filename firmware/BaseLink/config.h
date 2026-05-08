/*
 * ============================================================
 *  config.h — Self-Balancing Robot Configuration
 * ============================================================
 *  Central configuration for pin assignments, motor parameters,
 *  PID gains, filter coefficients, and safety limits.
 *
 *  All tunable constants live here. Implementation files do
 *  not require manual edits during a tuning session.
 * ============================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  PIN DEFINITIONS — ESP32 38-Pin DevKit
// ============================================================

// ---- Right Stepper Motor (TMC2208) ----
#define RIGHT_STEP_PIN    33
#define RIGHT_DIR_PIN     25
#define RIGHT_EN_PIN      32
#define RIGHT_UART_TX     17    // Requires a 1 kΩ inline resistor.
#define RIGHT_UART_RX     16

// ---- Left Stepper Motor (TMC2208) ----
#define LEFT_STEP_PIN     27
#define LEFT_DIR_PIN      14
#define LEFT_EN_PIN       26
#define LEFT_UART_TX      19    // Requires a 1 kΩ inline resistor.
#define LEFT_UART_RX      18

// ---- I2C Bus ----
#define I2C_SDA           21
#define I2C_SCL           22
#define I2C_CLOCK_HZ      400000  // 400 kHz fast-mode.
#define ONBOARD_LED       2

// ---- MT6816 ABZ Quadrature Encoders (PCNT) ----
#define ENC_LEFT_A         4
#define ENC_LEFT_B         5
#define ENC_RIGHT_A       13
#define ENC_RIGHT_B       23
#define ENCODER_PPR       1024    // MT6816 pulses per revolution.
#define ENCODER_CPR       (ENCODER_PPR * 4)  // 4096 counts/rev with 4x decode.
#define PCNT_H_LIM        30000
#define PCNT_L_LIM       -30000

// ============================================================
//  MOTOR / DRIVER PARAMETERS
// ============================================================
#define MICROSTEPS           16
#define STEPS_PER_REV        200      // Full steps/rev for a 1.8° motor.
#define USTEPS_PER_REV       (STEPS_PER_REV * MICROSTEPS)  // 3200 steps/rev.
#define MOTOR_CURRENT_MA     1100     // RMS current target via UART.
#define R_SENSE              0.11f    // Sense resistor on the TMC2208 PCB (Ω).

// Motors are mechanically mirrored; one axis must be inverted so that
// a positive PID output drives the robot forward.
#define RIGHT_DIR_INVERT     false
#define LEFT_DIR_INVERT      true

// ============================================================
//  STEPPER TIMER
// ============================================================
//  A hardware timer fires at TIMER_FREQ_HZ. A Bresenham accumulator
//  inside the ISR decides whether to pulse each motor on that tick.
//  Maximum step rate equals TIMER_FREQ_HZ.
//
//  At 20 kHz / 16 microsteps: 20000 / 3200 ≈ 6.25 rev/s (~375 RPM).
// ============================================================
#define TIMER_FREQ_HZ        20000
#define TIMER_PRESCALER       80      // Divides 80 MHz APB clock to 1 MHz base.
#define TIMER_ALARM_COUNT    (1000000 / TIMER_FREQ_HZ)

// ============================================================
//  BALANCE PID PARAMETERS
// ============================================================
#define DEFAULT_KP           1250.0f
#define DEFAULT_KI            0.0f
#define DEFAULT_KD            1.786f
#define DEFAULT_TARGET_ANGLE  0.071f    // Upright center-of-gravity offset (degrees).

// Output limits map to the stepper timer's maximum step rate.
#define PID_OUTPUT_MIN       -10000
#define PID_OUTPUT_MAX       10000

// Anti-windup clamp on the integral accumulator.
#define INTEGRAL_LIMIT        1500.0f

// Adaptive Kp — multiplies Kp when tilt error exceeds the threshold.
#define ADAPTIVE_ERROR_THRESHOLD 4.0f
#define ADAPTIVE_KP_BOOST        1.05f

// ============================================================
//  MANUAL DRIVING & POSITION HOLD
// ============================================================
#define MANUAL_DRIVE_RATE      3.0f     // Angle accumulation rate for W/S keyhold (°/s).
#define MAX_MANUAL_TILT        6.0f     // Maximum commanded lean angle (degrees).
#define BRAKE_DECAY_RATE       8.0f     // EMA decay rate when input returns to zero.

// Encoder-based drift correction.
#define MAX_POS_HOLD_TILT      1.5f     // Maximum lean authority for drift correction (degrees).
#define STEER_SMOOTHING        25.0f    // EMA ramp coefficient for steering input smoothing.

// ============================================================
//  IMU OUTPUT FILTER (2nd-order cascaded low-pass)
// ============================================================
// Higher cutoff = less lag, more noise. Lower cutoff = smoother, more lag.
#define IMU_FILTER_CUTOFF_HZ  50.0f

// ============================================================
//  MOTOR ACCELERATION LIMITER
// ============================================================
// Maximum speed change rate (steps/s²). Prevents stall on abrupt PID spikes.
// 60000 steps/s² = 300 steps/s change per 200 Hz tick.
#define MOTOR_ACCEL_LIMIT     60000.0f

// ============================================================
//  PID DERIVATIVE FILTER
// ============================================================
// First-order low-pass on the derivative term.
// Range: 0.0 (fully filtered) to 1.0 (unfiltered).
#define PID_D_FILTER_ALPHA    0.3f

// ============================================================
//  HEADING HOLD PID (YAW)
// ============================================================
#define YAW_KP               20.0f
#define YAW_KI               0.1f
#define YAW_KD               5.0f
#define MAX_STEERING         4000.0f  // Peak differential step split for yaw control.

// ============================================================
//  BLUETOOTH DRIVING
// ============================================================
#define DRIVE_SPEED_STEPS_SEC 6000.0f   // Forward drive target (steps/s).
#define TURN_SPEED_DEG_SEC    120.0f    // Peak yaw rate for A/D commands (°/s).

// ============================================================
//  AHRS / MAHONY FILTER
// ============================================================
//  Fuses accel, gyro, and magnetometer into a 9-DOF quaternion,
//  then decomposes it to pitch, roll, and heading angles.
// ============================================================
#define MAHONY_KP 2.0f    // Proportional gain — convergence speed toward gravity/north.
#define MAHONY_KI 0.005f  // Integral gain — compensates long-term gyro drift.

// ============================================================
//  MAGNETOMETER (QMC5883L)
// ============================================================
// Hard-iron offsets for local magnetic distortion calibration.
#define MAG_OFFSET_X  0.0f
#define MAG_OFFSET_Y  0.0f
#define MAG_OFFSET_Z  0.0f

// Axis sign corrections to align magnetometer with accelerometer frame.
#define MAG_SIGN_X  1
#define MAG_SIGN_Y  1
#define MAG_SIGN_Z  1

// Samples averaged during static gyro bias calibration.
#define GYRO_CAL_SAMPLES      500

// ============================================================
//  IMU AXIS MAPPING
// ============================================================
//  Maps software axes to the physical mounting orientation of the
//  ISM6HG256X on this chassis.
//
//  This chassis: Y = forward, Z = up.
//    Pitch tilt projects gravity onto Y.
//    Pitch rate circulates around X.
// ============================================================
#define PITCH_ACCEL_PRIMARY    'Y'    // Primary accel axis for tilt.
#define PITCH_ACCEL_SECONDARY  'Z'    // Orthogonal accel axis for atan2.
#define PITCH_ACCEL_SIGN        -1    // Sign correction (1 or -1).
#define PITCH_GYRO_AXIS        'X'    // Gyro axis recording pitch rate.
#define PITCH_GYRO_SIGN         -1    // Sign correction (1 or -1).

// ============================================================
//  IMU SENSOR SETTINGS
// ============================================================
#define IMU_ODR_HZ             960.0f   // Output data rate (Hz).
#define IMU_ACCEL_FS           4        // Accelerometer full-scale (±g).
#define IMU_GYRO_FS            2000     // Gyroscope full-scale (±°/s).

// ============================================================
//  SAFETY
// ============================================================
#define MAX_TILT_ANGLE        95.0f   // Tilt angle that triggers motor cutoff (degrees).
#define STARTUP_SETTLE_MS     3000    // Sensor warm-up delay before balancing starts (ms).

// ============================================================
//  CONTROL LOOP TIMING
// ============================================================
#define LOOP_FREQ_HZ          200
#define LOOP_PERIOD_US        (1000000UL / LOOP_FREQ_HZ)

// ============================================================
//  SERIAL / DEBUG
// ============================================================
#define SERIAL_BAUD           115200
#define PLOT_DIVIDER          10      // Print every Nth loop: 200 Hz / 10 = 20 Hz output.

#endif // CONFIG_H
