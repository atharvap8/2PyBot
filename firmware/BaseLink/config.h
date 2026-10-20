/* 2PyBot v4.0.3: ESP32-WROOM + two M0601 RS485 BLDC hubs.
 * Control state is robot-forward SI; the wire layer applies motor polarity.
 * Pinout and geometry follow the supplied BaseLinkHub program.
 */
#ifndef CONFIG_H
#define CONFIG_H

#define RIGHT_MOTOR_ID       2
#define RIGHT_RS485_TX       19
#define RIGHT_RS485_RX       18
#define RIGHT_RS485_DE       23
#define LEFT_MOTOR_ID        1
#define LEFT_RS485_TX        17
#define LEFT_RS485_RX        16
#define LEFT_RS485_DE        4
#define RIGHT_MOTOR_DIR      (+1.0f)
#define LEFT_MOTOR_DIR       (-1.0f)
// Confirmed by the owner. Not inherited from either old NVS namespace.
#define HUB_CURRENT_SIGN    (-1.0f)
#define HUB_RPM_SIGN        (-1.0f)

#define I2C_SDA             21
#define I2C_SCL             22
#define I2C_CLOCK_HZ        400000
#define SUPPLY_V            12.0f
#define M0601_KT            0.75f
#define M0601_KV_RPM_PER_V   11.1f
#define M0601_CPR           4096
#define ENCODER_CPR         M0601_CPR
#define MOTOR_CURRENT_MA    2700
#define MOTOR_CURRENT_MAX_MA 4400
#define HUB_DRIVE_MODE      1
#define HUB_VEL_ACCEL_BYTE  0
#define HUB_LINK_TIMEOUT_MS 60

// Lower sampled velocity-loop gains reduce the current reversals observed
// under load with delayed position feedback. Full torque remains available.
#define HUB_KP              4.0f
#define HUB_KI              4.0f
#define HUB_INT_LIM         0.5f
#define HUB_KFF             1.0f
#define HUB_MASS_KG         2.0f
#define HUB_VEL_SOURCE      1 // owner-confirmed fix: position-derived velocity
#define HUB_YAW_KP          1.5f
#define HUB_YAW_KI          2.0f
#define HUB_YAW_INT_LIM     0.35f
#define HUB_YAW_CURRENT_SLEW 6.0f

#define WHEEL_RADIUS_M      0.0520f
#define COUNTS_PER_M        ((float)ENCODER_CPR / (2.0f * PI * WHEEL_RADIUS_M))
#define STEPS_PER_M         COUNTS_PER_M // retained protocol name; counts, not microsteps
#define TRACK_WIDTH_M       0.155f
#define PITCH_FWD_SIGN      (-1.0f)
#define ENC_FWD_SIGN        (+1.0f)
#define RATE_FWD_SIGN       (+1.0f)
#define PITCH_TRIM_DEG      1.54042f

// Keep the supplied LQI design; the actuator loop and speed windup are
// corrected separately rather than guessing new body gains without a log.
#define LQR_K1              (-7.3234f)
#define LQR_K2              (-7.4495f)
#define LQR_K3              (-42.9506f)
#define LQR_K4              (-6.6862f)
#define LQR_K5              (-2.1082f)
#define Z_INT_LIM           0.5f
#define EX_CLAMP_M          0.15f
#define VSAT_BLEED          3.0f
#define V_HOLD_MAX_MS       0.35f
#define BRAKE_LOOKAHEAD_S    0.30f
#define LQR_S1              (-60.0226f)
#define LQR_S2              (-31.0364f)
#define LQR_S3              (-82.0431f)
#define LQR_S4              (-10.2571f)
#define LQR_S5              (-25.8199f)
#define LQR_C1              (-22.2369f)
#define LQR_C2              (-20.0745f)
#define LQR_C3              (-81.3063f)
#define LQR_C4              (-12.7029f)
#define LQR_C5              (-8.1650f)
#define CLIMB_Z_INT_LIM     2.0f
#define CLIMB_VEL_MS        0.25f
#define CLIMB_STEER_SCALE   0.40f
#define LOOK_MAX_DEG        18.0f

// MAXSPD=0 is uncapped. The voltage/free-speed model is telemetry only.
#define MAX_SPEED_STEPS     0.0f
#define MOTOR_ACCEL_LIMIT   (6.0f * COUNTS_PER_M)
#define MAX_DRIVE_VEL_MS     1.00f   // requested; limited by SPEED_HEADROOM x free speed
// Share of the motor free speed usable as a drive reference. The rest is the
// catch margin the balance loop needs (simulation: 0.70 survives a shove at
// top speed, 0.80 does not). Not a diagnostic cap: measure FREERPM (AT,stall)
// or raise the supply voltage to raise the real top speed.
#define SPEED_HEADROOM       0.70f
#define DRIVE_DEADBAND      0.03f
#define JOY_FWD_SCALE       (-0.2f)
#define JOY_STEER_SCALE     1.0f
#define MAX_STEER_STEPS     (1.5f * TRACK_WIDTH_M * COUNTS_PER_M * 0.5f)
#define STEER_SLEW          (4.0f * TRACK_WIDTH_M * COUNTS_PER_M * 0.5f)
#define STEER_DEADBAND      0.05f
#define YAW_HOLD_KP         1.5f
#define YAW_HOLD_KD         0.03f
#define YAW_HOLD_MAX_STEPS  (0.8f * TRACK_WIDTH_M * COUNTS_PER_M * 0.5f)
#define YAW_RELATCH_COUNTS  2000.0f
#define YAW_TILT_SUSPEND_DEG 12.0f

#define RADXA_TIMEOUT_MS    600
#define JOY_TIMEOUT_MS_CFG  200
#define HOST_BOOT_WAIT      1.0f
#define ARM_ANGLE_DEG       5.0f
#define MAX_TILT_ANGLE      55.0f
#define MAX_PITCH_RATE_SAFETY 450.0f
#define STARTUP_SETTLE_MS   3000
#define IMU_FILTER_CUTOFF_HZ 50.0f
#define MAHONY_KP           2.0f
#define MAHONY_KI           0.005f
#define MAG_OFFSET_X        0.0f
#define MAG_OFFSET_Y        0.0f
#define MAG_OFFSET_Z        0.0f
#define MAG_SIGN_X          1
#define MAG_SIGN_Y          1
#define MAG_SIGN_Z          1
#define GYRO_CAL_SAMPLES    500
#define PITCH_ACCEL_PRIMARY 'Y'
#define PITCH_ACCEL_SECONDARY 'Z'
#define PITCH_ACCEL_SIGN    -1
#define PITCH_GYRO_AXIS     'X'
#define PITCH_GYRO_SIGN     -1
#define IMU_ODR_HZ          960.0f
#define IMU_ACCEL_FS        4
#define IMU_GYRO_FS         2000
#define LOOP_FREQ_HZ        200
#define LOOP_PERIOD_US      (1000000UL / LOOP_FREQ_HZ)
#define SERIAL_BAUD         460800
#define ODOM_PERIOD_MS      20
#define DEBUG_PERIOD_MS     20

#define LED_RING_PIN        15
#define LED_RING_COUNT      16
#define LED_FRONT_INDEX     7.5f
#define LED_DIR_CW          1
#define LED_MAX_BRIGHT      200
#define LED_FPS             50
#define PAD_FWD_SIGN        (+1.0f)
#define PAD_STEER_SIGN      (-1.0f)
#define PAD_DEADZONE        0.08f
#define STICK_DEADZONE      0.10f
#define SNAP_IN_DEG         10.0f
#define SNAP_OUT_DEG        26.0f
#define EXPO_STEER          0.65f
#define EXPO_DRIVE          0.35f
#define STICK_AUTOCAL       1
#define SPEED_LO_DRIVE_SCALE 0.50f
#define SPEED_LO_STEER_SCALE 0.50f
#define SPEED_BOOT_HIGH     0

// Supplied BLDC harness: pan moved to 26; zoom is 0, torch remains 2.
#define SERVO_YAW_PIN       26
#define SERVO_ZOOM_PIN      0
#define TORCH_PIN           2
#define SERVO_YAW_CH        4
#define SERVO_ZOOM_CH       5
#define TORCH_CH            6
#define SERVO_PWM_HZ        50
#define SERVO_PWM_BITS      16
#define TORCH_PWM_HZ        20000
#define TORCH_PWM_BITS      10
#define SERVO_US_MIN        500.0f
#define SERVO_US_MAX        2400.0f
#define SERVO_YAW_MIN_DEG   120
#define SERVO_YAW_MAX_DEG   190
#define SERVO_YAW_HOME_DEG  165
#define SERVO_ZOOM_MIN_DEG  80
#define SERVO_ZOOM_MAX_DEG  140
#define SERVO_ZOOM_HOME_DEG 120
#define SERVO_YAW_RATE_DPS  70.0f
#define SERVO_ZOOM_RATE_DPS 45.0f
#define SERVO_UPDATE_HZ     50
#define SERVO_IDLE_RELEASE_MS 0
#define TORCH_MAX_PCT       90
#define TORCH_MIN_PCT       5
#define TORCH_BOOT_PCT      25
#define TORCH_STEP_PCT      10
#define TORCH_OFF_ON_DISARM 0
#define PAD_YAW_SIGN        (-1.0f)
#define PAD_ZOOM_SIGN       (+1.0f)
#define PAD_REPEAT_FIRST_MS 400
#define PAD_REPEAT_MS       130
#endif
