/*
 * ============================================================
 *  imu_sensor.cpp — ISM6HG256X implementation
 * ============================================================
 *  Provides the executable definitions for the IMUSensor class,
 *  orchestrating the physical hardware I2C transactions, 
 *  mathematical zero-calibration routines, and the continuous
 *  execution of the Mahony AHRS fusion algorithm.
 * ============================================================
 */

#include "imu_sensor.h"

// ============================================================
//  Constructor
// ============================================================
// Initializes object attributes with safe default values.
// Binds the ISM6HG256X sensor object to the primary `Wire` 
// I2C bus handler.
IMUSensor::IMUSensor()
    : _sensor(&Wire),
      _pitch(0.0f), _yaw(0.0f), _pitchRate(0.0f), _accelAngle(0.0f),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      eInt_x(0.0f), eInt_y(0.0f), eInt_z(0.0f),
      _ax(0), _ay(0), _az(0),
      _gx(0), _gy(0), _gz(0),
      _mx(0), _my(0), _mz(0),
      _gyroOffX(0), _gyroOffY(0), _gyroOffZ(0),
      _firstReading(true),
      _filtStage1(0.0f), _filtStage2(0.0f), _filtAlpha(0.0f),
      _filterSeeded(false)
{}

// ============================================================
//  begin()
// ============================================================
// Validates physical hardware presence, configures operational modes,
// scales resolutions, and establishes algorithmic prerequisites.
bool IMUSensor::begin() {
    
    // Initialize the primary I2C bus physically attached to the IMU.
    Wire.begin(I2C_SDA, I2C_SCL);
    // Overclock the standard interface to 400 kHz to accommodate 200 Hz PID execution.
    Wire.setClock(I2C_CLOCK_HZ);

    // Bootstrap the discrete sensor library logic.
    if (_sensor.begin() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: begin() failed — check wiring / address");
        return false;
    }

    // Ping the sensor for its fixed internal manufacturer ID to confirm integrity.
    uint8_t id;
    if (_sensor.ReadID(&id) == ISM6HG256X_OK) {
        Serial.printf("[IMU] Sensor ID: 0x%02X\n", id);
    }

    // ---- Accelerometer Initialization ----
    // Enable the accelerometer data pipeline energetically.
    if (_sensor.Enable_X() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Enable_X failed");
        return false;
    }
    // Command the maximum stable high-performance output data rate.
    if (_sensor.Set_X_OutputDataRate_With_Mode(
            IMU_ODR_HZ,
            ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set accel ODR failed");
        return false;
    }
    // Assign the full-scale sensitivity range defined within config.h.
    if (_sensor.Set_X_FullScale(IMU_ACCEL_FS) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set accel FS failed");
        return false;
    }
    Serial.printf("[IMU] Accel: %.0f Hz ODR, ±%d g, High-Performance\n",
                  IMU_ODR_HZ, IMU_ACCEL_FS);

    // ---- Gyroscope Initialization ----
    // Enable the separate gyroscope data pipeline energetically.
    if (_sensor.Enable_G() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Enable_G failed");
        return false;
    }
    // Bind the gyroscope sample frequency identically to the accelerometer.
    if (_sensor.Set_G_OutputDataRate_With_Mode(
            IMU_ODR_HZ,
            ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set gyro ODR failed");
        return false;
    }
    // Assign rotational sensitivity (degrees per second max).
    if (_sensor.Set_G_FullScale(IMU_GYRO_FS) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set gyro FS failed");
        return false;
    }
    Serial.printf("[IMU] Gyro:  %.0f Hz ODR, ±%d °/s, High-Performance\n",
                  IMU_ODR_HZ, IMU_GYRO_FS);

    // ---- Magnetometer Initialization ----
    // Awaken the supplementary QMC5883L heading sensor locally.
    _compass.init();
    Serial.println("[IMU] QMC5883L init called");
    Serial.println("[IMU] Initialisation OK");

    // ---- Filter Mathematics ----
    // Precalculate the 2nd-order cascaded low-pass filter coefficient natively.
    // The equation derives alpha from an exact discretization of a 1st-order RC low-pass.
    // Utilizing precomputation here saves extensive division operations inside the 200 Hz loop.
    _filtAlpha = 1.0f - expf(-2.0f * M_PI * IMU_FILTER_CUTOFF_HZ / LOOP_FREQ_HZ);
    Serial.printf("[IMU] Pitch filter: %.1f Hz cutoff, alpha=%.4f\n",
                  IMU_FILTER_CUTOFF_HZ, _filtAlpha);
                  
    return true;
}

// ============================================================
//  calibrateGyro()
// ============================================================
// Performs a zero-rate ambient noise profiling of the gyroscope hardware.
// The algorithm logs 500 consecutive readings over several seconds while
// completely stationary, creating baseline reference offsets subtracted 
// forever hereafter during dynamic flight operations.
bool IMUSensor::calibrateGyro() {
    Serial.printf("[IMU] Calibrating gyro (%d samples) — keep robot still!\n",
                  GYRO_CAL_SAMPLES);

    float sumX = 0, sumY = 0, sumZ = 0;
    ISM6HG256X_Axes_t g;
    int good = 0;

    // Accumulate consecutive samples using a strict temporal delay to guarantee independent metrics.
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (_sensor.Get_G_Axes(&g) == ISM6HG256X_OK) {
            sumX += g.x;
            sumY += g.y;
            sumZ += g.z;
            good++;
        }
        // Force a 2 ms delay yielding an effective 500 Hz calibration sampling rhythm.
        delay(2);
    }

    // Validate if sufficient independent read actions executed successfully over the bus.
    if (good < GYRO_CAL_SAMPLES / 2) {
        Serial.println("[IMU] ERROR: too few good readings during calibration");
        return false;
    }

    // Average the cumulative offsets explicitly in milli-degrees per second (mdps).
    _gyroOffX = sumX / good;
    _gyroOffY = sumY / good;
    _gyroOffZ = sumZ / good;

    Serial.printf("[IMU] Gyro offsets (mdps): X=%.1f  Y=%.1f  Z=%.1f\n",
                  _gyroOffX, _gyroOffY, _gyroOffZ);
    return true;
}

// ============================================================
//  update()  — Read sensors + Complementary Filter
// ============================================================
// Retrieves raw hardware vectors, standardizes the scales, executes
// vibration compensation, runs the discrete Mahony algorithmic fusion,
// and enforces digital second-order noise constraints.
void IMUSensor::update(float dt) {
    ISM6HG256X_Axes_t accelRaw, gyroRaw;

    // Attempt hardware data extraction synchronously. Abort processing purely 
    // to preserve previous steady-state logic if the I2C bus temporarily fails.
    if (_sensor.Get_X_Axes(&accelRaw) != ISM6HG256X_OK) return;
    if (_sensor.Get_G_Axes(&gyroRaw)  != ISM6HG256X_OK) return;

    // --- Format conversions ---
    // Translate raw milligravity integer data into clean acceleration factors strictly aligned to 1-G.
    _ax = accelRaw.x / 1000.0f;
    _ay = accelRaw.y / 1000.0f;
    _az = accelRaw.z / 1000.0f;

    // Translate gyroscope integers, subtract the pre-calculated calibration baseline,
    // and format finally into explicit degrees per second.
    _gx = (gyroRaw.x - _gyroOffX) / 1000.0f;
    _gy = (gyroRaw.y - _gyroOffY) / 1000.0f;
    _gz = (gyroRaw.z - _gyroOffZ) / 1000.0f;

    // --- Crude Fallback Tilt Estimation ---
    // Derive primitive tilt using simple geometrical atan2 operations utilizing strictly gravity vectors.
    // Extremely responsive to static position but mathematically disrupted entirely by physical translation (driving).
    float aPri = axisValue(accelRaw, PITCH_ACCEL_PRIMARY)   / 1000.0f;
    float aSec = axisValue(accelRaw, PITCH_ACCEL_SECONDARY) / 1000.0f;
    _accelAngle = atan2f(aPri, aSec) * RAD_TO_DEG * PITCH_ACCEL_SIGN;

    // --- Independent Rotational Extraction ---
    // Extract logical degree-per-second constraints relative entirely to the localized pitch configuration axis.
    float gyroPitch = axisValue(gyroRaw, PITCH_GYRO_AXIS);
    switch (PITCH_GYRO_AXIS) {
        case 'X': gyroPitch -= _gyroOffX; break;
        case 'Y': gyroPitch -= _gyroOffY; break;
        case 'Z': gyroPitch -= _gyroOffZ; break;
    }
    _pitchRate = (gyroPitch / 1000.0f) * PITCH_GYRO_SIGN;

    // --- External Magnetometer Data Mapping ---
    // Extract ambient magnetic vectors and eliminate predefined chassis metallic offsets.
    _compass.read();
    _mx = (_compass.getX() - MAG_OFFSET_X) * MAG_SIGN_X;
    _my = (_compass.getY() - MAG_OFFSET_Y) * MAG_SIGN_Y;
    _mz = (_compass.getZ() - MAG_OFFSET_Z) * MAG_SIGN_Z;

    // --- Standardized Mahony AHRS Algorithm Structure ---
    // Transition inputs internally representing global state mathematics.
    float ax = _ax, ay = _ay, az = _az;
    float mx = _mx, my = _my, mz = _mz;
    
    // Scale gyroscope values completely from degrees/second to purely analytical radians/second.
    float gx = _gx * DEG_TO_RAD, gy = _gy * DEG_TO_RAD, gz = _gz * DEG_TO_RAD;

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Pre-calculate recurrent fractional multiplication chains improving processor efficiency.
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Proportional fusion execution boundary: Bypass entirely if an I2C error registered an impossibly zeroed acceleration vector.
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalize accelerometer vector length explicitly to 1 unit.
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimate the current orientation of Earth's gravity based on internal quaternion tracking.
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // Calculates the error represented as a cross-product difference between physical gravity 
        // measured vs theoretical gravity tracked.
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        
        // --- Dynamic Acceleration Vibration Rejection ---
        // Measures the absolute magnitude of physical acceleration actively experienced by the robot.
        // A perfect stationary entity measures identically 1.0G.
        float accelMagGs = sqrtf(_ax*_ax + _ay*_ay + _az*_az);
        float currentKp = MAHONY_KP;
        
        // If a significant impact, jump, or aggressive drive maneuver generates massive linear forces,
        // drastically lower the Kp multiplier. This mathematically tells the filter temporarily to "ignore the 
        // confused accelerometer and completely trust the stable gyroscope" until the linear disruption subsides.
        if (fabsf(accelMagGs - 1.0f) > 0.2f) {
            currentKp = 0.1f;
        }

        // Incrementally accumulate the integral error metric to passively compensate for gyroscope temperature drift. 
        if(MAHONY_KI > 0.0f) {
            eInt_x += MAHONY_KI * halfex * dt;
            eInt_y += MAHONY_KI * halfey * dt;
            eInt_z += MAHONY_KI * halfez * dt;
            gx += eInt_x;
            gy += eInt_y;
            gz += eInt_z;
        }

        // Enforce the calculated proportional correction directly back against the mathematical gyroscopic inputs.
        gx += currentKp * halfex;
        gy += currentKp * halfey;
        gz += currentKp * halfez;
    }

    // Synthesize the fundamental rate-of-change across the full quaternion.
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize the final quaternion result structurally to maintain unit length mathematical consistency.
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // --- Complex Geometric Disassembly ---
    // Deconstruct the abstract 4D quaternion back exclusively into standard Euler angles representing Pitch and Roll.
    float roll_ahrs  = atan2f(q0*q1 + q2*q3, 0.5f - q1q1 - q2q2) * RAD_TO_DEG;
    float pitch_ahrs = asinf(-2.0f * (q1*q3 - q0*q2)) * RAD_TO_DEG;
    
    // --- Tilt-Compensated Yaw Formulation ---
    // Project the 2D ambient magnetometer readings firmly onto the 3D surface plane calculated above.
    // This totally isolates directional logic (Heading) from the chaotic angular pitch oscillations produced constantly 
    // by balancing mechanics, ensuring the robot drives perfectly straight regardless of lean angle.
    float rRad = roll_ahrs * DEG_TO_RAD;
    float pRad = pitch_ahrs * DEG_TO_RAD;
    float Xh = mx * cosf(pRad) + my * sinf(rRad) * sinf(pRad) - mz * cosf(rRad) * sinf(pRad);
    float Yh = my * cosf(rRad) + mz * sinf(rRad);
    
    // Extract final Yaw calculation specifically from the stabilized magnetic plane.
    _yaw = atan2f(Yh, Xh) * RAD_TO_DEG;
    
    // Route the final designated Pitch dimension dynamically towards PID operations based exclusively 
    // on user-defined configuration mapping. 
    if (PITCH_GYRO_AXIS == 'X' || PITCH_GYRO_AXIS == 'x') {
        _pitch = roll_ahrs * (PITCH_GYRO_SIGN * PITCH_ACCEL_SIGN > 0 ? 1 : -1); 
    } else if (PITCH_GYRO_AXIS == 'Y' || PITCH_GYRO_AXIS == 'y') {
        _pitch = pitch_ahrs * (PITCH_GYRO_SIGN * PITCH_ACCEL_SIGN > 0 ? 1 : -1);
    } else {
        _pitch = _accelAngle; // Hard fallback if improperly configured logic prevails.
    }

    // --- 2nd-Order Cascaded Filtering Sequence ---
    // Implementation of two sequential Exponential Moving Average filters running in discrete series.
    // Generates an incredibly clean -40 dB/decade structural drop-off isolating only fundamental chassis 
    // angles while efficiently annihilating transient, physical vibrational spikes caused by running steppers.
    
    // Actuates a logical seed initialization eliminating mathematical transient drag when evaluating the first cycle.
    if (!_filterSeeded) {
        _filtStage1 = _pitch;
        _filtStage2 = _pitch;
        _filterSeeded = true;
    } else {
        _filtStage1 += _filtAlpha * (_pitch - _filtStage1);
        _filtStage2 += _filtAlpha * (_filtStage1 - _filtStage2);
        _pitch = _filtStage2;
    }
}

// ============================================================
//  axisValue()  — Mapping Abstraction Helper
// ============================================================
// Returns corresponding explicit float value referencing generic 
// structural data objects dynamically using fundamental character definitions.
float IMUSensor::axisValue(const ISM6HG256X_Axes_t& d, char axis) {
    switch (axis) {
        case 'X': case 'x': return (float)d.x;
        case 'Y': case 'y': return (float)d.y;
        case 'Z': case 'z': return (float)d.z;
        default:             return 0.0f;
    }
}
