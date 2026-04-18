/*
 * ============================================================
 *  config.h — Self-Balancing Robot Configuration
 * ============================================================
 *  Central configuration for pin assignments, motor parameters,
 *  PID gains, filter coefficients, and safety limits.
 *
 *  All tunable constants are placed here so that implementation
 *  files do not require manual editing during a tuning session.
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
#define RIGHT_UART_TX     17    // Requires a 1 kΩ inline resistor on the TX line.
#define RIGHT_UART_RX     16

// ---- Left Stepper Motor (TMC2208) ----
#define LEFT_STEP_PIN     27
#define LEFT_DIR_PIN      14
#define LEFT_EN_PIN       26
#define LEFT_UART_TX      19    // Requires a 1 kΩ inline resistor on the TX line.
#define LEFT_UART_RX      18

// ---- I2C Bus (default Wire) ----
#define I2C_SDA           21
#define I2C_SCL           22
#define I2C_CLOCK_HZ      400000  // 400 kHz Fast-mode capability configuration.
#define ONBOARD_LED       2

// ============================================================
//  MOTOR / DRIVER PARAMETERS
// ============================================================
#define MICROSTEPS           16       // Software microstep configuration for the TMC2208 drivers.
#define STEPS_PER_REV        200      // Full physical steps per revolution for a standard 1.8-degree motor.
#define USTEPS_PER_REV       (STEPS_PER_REV * MICROSTEPS)  // Equals 3200 steps per revolution.
#define MOTOR_CURRENT_MA     1100     // Maximum RMS current target delivered to the TMC2208 via UART.
#define R_SENSE              0.11f    // Rated sense resistor value on the specific TMC2208 module PCB (measured in Ohms).

// Direction inversion logic.
// Because the stepper motors physically mirror each other on the chassis, one must be inverted electrically.
// This ensures that a positive aggregate PID output successfully drives the robot forwards.
#define RIGHT_DIR_INVERT     false
#define LEFT_DIR_INVERT      true

// ============================================================
//  STEPPER TIMER
// ============================================================
//  A high-speed hardware timer fires at the configured TIMER_FREQ_HZ. 
//  Inside the underlying Interrupt Service Routine (ISR), a Bresenham 
//  accumulator algorithm decides whether to pulse each motor on that specific tick.
//  Therefore, the absolute maximum achievable step rate equals TIMER_FREQ_HZ.
//
//  For example: A 20 kHz interrupt translates to a 50 µs timing period, yielding a max of 20,000 steps/s.
//  At 16 microsteps, this results in: 20000 / 3200 ≈ 6.25 rev/s (approximately 375 RPM), which is adequate.
// ============================================================
#define TIMER_FREQ_HZ        20000
#define TIMER_PRESCALER       80      // Divides the 80 MHz APB system clock by 80 to establish a 1 MHz base tick.
#define TIMER_ALARM_COUNT    (1000000 / TIMER_FREQ_HZ)  // Derives the specific alarm interrupt boundary count.

// ============================================================
//  BALANCE PID PARAMETERS
// ============================================================
#define DEFAULT_KP           1000.0f
#define DEFAULT_KI            30.0f
#define DEFAULT_KD            30.0f
#define DEFAULT_TARGET_ANGLE  -1.469f    // The naturally settled upright center of gravity offset (measured in degrees).

// PID Output limits map directly to the maximum capability of the stepper timer routine (steps per second).
#define PID_OUTPUT_MIN       (-((float)TIMER_FREQ_HZ))
#define PID_OUTPUT_MAX       ((float)TIMER_FREQ_HZ)

// Prevents integral windup accumulation during long mechanical stalls.
#define INTEGRAL_LIMIT        5000.0f

// Adaptive PID multipliers scale specific coefficients conditionally when the robot falls deeply out of balance.
#define ADAPTIVE_ERROR_THRESHOLD 2.0f   // The dynamic tilt angle threshold causing Kp to multiply.
#define ADAPTIVE_KP_BOOST        1.5f   // The mathematical multiplier applied to Kp when the threshold is exceeded.

// ============================================================
//  MANUAL DRIVING & POSITION HOLD
// ============================================================
#define MANUAL_DRIVE_RATE      3.0f     // Rate of angle accumulation (degrees per second) when receiving a W/S keyhold command.
#define MAX_MANUAL_TILT        6.0f     // Absolute maximum positional lean angle command constraint allowed for safety.
#define BRAKE_DECAY_RATE       8.0f     // Exponential damping rate applied mathematically to angular velocity upon returning to zero input.

// Position Maintenance Algorithm (stepper-odometry drift correction).
#define POS_HOLD_KP            0.0006f  // Factor translating raw stepped physical distance error back into correcting angular influence.
#define POS_HOLD_KD            0.003f   // Derivative factor dampening physical sway while maintaining position.
#define MAX_POS_HOLD_TILT      3.0f     // Peak pitch limit angle authority allocated specifically for autonomous positional corrections.
#define STEER_SMOOTHING        25.0f    // Mathematical ramp time coefficient mitigating instantaneous mechanical shocks when initiating a turn.

// ============================================================
//  IMU OUTPUT FILTER (2nd-order cascaded low-pass)
// ============================================================
// Cutoff frequency in Hz. Actuates exponential averaging in two cascading sequences.
// An elevated cutoff rate (e.g. 50Hz) resolves phase-lag instability problems by improving input responsiveness.
#define IMU_FILTER_CUTOFF_HZ  50.0f

// ============================================================
//  MOTOR ACCELERATION LIMITER
// ============================================================
// Determines the maximum allowable stepper speed rate change, categorized mathematically in steps/sec^2.
// Prevents total electromechanical starvation (stalling) during sudden PID speed spikes.
// A limit of 200,000 correlates to a maximum discrete variation of 1,000 velocity units per 200Hz evaluation tick.
#define MOTOR_ACCEL_LIMIT     200000.0f

// ============================================================
//  PID DERIVATIVE FILTER
// ============================================================
// Dedicated low-pass signal filter isolated explicitly to the reactive Derivative component output.
// Lowering the 'Alpha' coefficient mitigates high-frequency IMU signal noise but intrinsically introduces reactive phase lag.
// Values range between 0.0 (frozen differential) and 1.0 (unfiltered transmission).
#define PID_D_FILTER_ALPHA    0.7f

// ============================================================
//  HEADING HOLD PID PARAMETERS (YAW)
// ============================================================
#define YAW_KP               20.0f
#define YAW_KI               0.1f
#define YAW_KD               5.0f
#define MAX_STEERING         4000.0f  // Defines the peak allowable differential split-step output allocated for yaw control.

// ============================================================
//  BLUETOOTH DRIVING CONSTANTS
// ============================================================
#define DRIVE_SPEED_STEPS_SEC 6000.0f   // The theoretical ghost-target pursuit pace when employing purely autonomous forward movement algorithms.
#define TURN_SPEED_DEG_SEC    120.0f    // The peak angular momentum target assigned when executing a rotation command via A/D keyboard input.

// ============================================================
//  AHRS / MAHONY FILTER
// ============================================================
//  The integrated Mahony algorithm consumes raw vectors from the accelerometer, gyroscope,
//  and accompanying magnetometer to fuse a highly resolute, 9 Degrees of Freedom orientation quaternion.
//  This geometric construct is then functionally decomposed back into pitch, roll, and heading planes.
// ============================================================
#define MAHONY_KP 2.0f    // The prominent proportional gain guiding global convergence rate towards magnetic north / absolute gravity.
#define MAHONY_KI 0.005f  // The integral accumulator gain dedicated internally to rectifying long-term static drift specific to the gyroscope.

// ============================================================
//  MAGNETOMETER (QMC5883L) PARAMETERS
// ============================================================
// Standard offset variables for calibrating local magnetic distortions specific to the chassis environment.
#define MAG_OFFSET_X  0.0f
#define MAG_OFFSET_Y  0.0f
#define MAG_OFFSET_Z  0.0f

// Mathematical inversion flags aligning the logical magnetic fields with the primary accelerometer coordinate structure.
#define MAG_SIGN_X  1
#define MAG_SIGN_Y  1
#define MAG_SIGN_Z  1

// Total amount of sampled evaluations gathered dynamically on initial boot to synthesize the average localized gyroscope bias.
#define GYRO_CAL_SAMPLES      500

// ============================================================
//  IMU AXIS MAPPING
// ============================================================
//  Adjusts the algorithmic processing directions to perfectly match the
//  precise physical mounting orientation of the ISM6HG256X module hardware on the robot chassis.
//
//  Configuration specific to this chassis variant: Y axis extends Forward; Z axis extends vertically Upwards.
//      Pitching tilt dynamically projects the primary gravity vector onto the localized Y axis.
//      Pitching rotational velocity circulates primarily around the localized lateral X axis.
// ============================================================
#define PITCH_ACCEL_PRIMARY    'Y'    // Directs the algorithm to the specific positional data containing raw primary tilt variations.
#define PITCH_ACCEL_SECONDARY  'Z'    // Defines the orthogonal measurement axis dedicated for stabilization.
#define PITCH_ACCEL_SIGN        -1    // Flips mathematical evaluation parity internally so physical rotation precisely equates to logical variation (1 or -1).
#define PITCH_GYRO_AXIS        'X'    // Exposes the independent gyroscope orientation axis recording the pitch velocity.
#define PITCH_GYRO_SIGN         -1    // Flips evaluation parity for the gyroscope specific vectors.

// ============================================================
//  IMU SENSOR SETTINGS
// ============================================================
#define IMU_ODR_HZ             960.0f   // Output Data Rate evaluation frequency established globally exclusively on the physical component level.
#define IMU_ACCEL_FS           4        // Sets the internal operational full-scale structural range dynamically up to ±4 forces of gravity.
#define IMU_GYRO_FS            2000     // Establishes maximum recordable rotational capacity boundaries directly at ±2000 angular degrees per second.

// ============================================================
//  SAFETY
// ============================================================
#define MAX_TILT_ANGLE        95.0f   // The peak physical boundary limit triggering the automatic motor disconnection emergency cutoff.
#define STARTUP_SETTLE_MS     3000    // Artificial time delay initializing core component logic prior to beginning balance functions.

// ============================================================
//  CONTROL LOOP TIMING
// ============================================================
#define LOOP_FREQ_HZ          200     // Central control loop execution rate mapping hardware inputs natively to PID functions.
#define LOOP_PERIOD_US        (1000000UL / LOOP_FREQ_HZ)  // Derived mathematical duration representing a single 200Hz time frame evaluating in microseconds.

// ============================================================
//  SERIAL / DEBUG
// ============================================================
#define SERIAL_BAUD           115200
#define PLOT_DIVIDER          10      // Instructs the serial printing function to intentionally skip evaluations. Example: 200 Hz / 10 = an efficient 20 Hz output stream.

#endif  // CONFIG_H
