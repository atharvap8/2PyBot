/*
 * ============================================================
 *  imu_sensor.cpp — ISM6HG256X implementation
 * ============================================================
 *  Provides definitions for the IMUSensor class: I2C transactions,
 *  gyro zero-rate calibration, and the Mahony AHRS fusion loop.
 * ============================================================
 */

#include "imu_sensor.h"

// ============================================================
//  Constructor
// ============================================================
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
bool IMUSensor::begin() {

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK_HZ);  // 400 kHz to support 200 Hz PID loop.

    if (_sensor.begin() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: begin() failed — check wiring / address");
        return false;
    }

    uint8_t id;
    if (_sensor.ReadID(&id) == ISM6HG256X_OK) {
        Serial.printf("[IMU] Sensor ID: 0x%02X\n", id);
    }

    // ---- Accelerometer ----
    if (_sensor.Enable_X() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Enable_X failed");
        return false;
    }
    if (_sensor.Set_X_OutputDataRate_With_Mode(
            IMU_ODR_HZ, ISM6HG256X_ACC_HIGH_PERFORMANCE_MODE) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set accel ODR failed");
        return false;
    }
    if (_sensor.Set_X_FullScale(IMU_ACCEL_FS) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set accel FS failed");
        return false;
    }
    Serial.printf("[IMU] Accel: %.0f Hz ODR, ±%d g, High-Performance\n",
                  IMU_ODR_HZ, IMU_ACCEL_FS);

    // ---- Gyroscope ----
    if (_sensor.Enable_G() != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Enable_G failed");
        return false;
    }
    if (_sensor.Set_G_OutputDataRate_With_Mode(
            IMU_ODR_HZ, ISM6HG256X_GYRO_HIGH_PERFORMANCE_MODE) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set gyro ODR failed");
        return false;
    }
    if (_sensor.Set_G_FullScale(IMU_GYRO_FS) != ISM6HG256X_OK) {
        Serial.println("[IMU] ERROR: Set gyro FS failed");
        return false;
    }
    Serial.printf("[IMU] Gyro:  %.0f Hz ODR, ±%d °/s, High-Performance\n",
                  IMU_ODR_HZ, IMU_GYRO_FS);

    // ---- Magnetometer ----
    _compass.init();
    Serial.println("[IMU] QMC5883L init called");
    Serial.println("[IMU] Initialisation OK");

    // Precompute filter alpha from the RC low-pass discretization formula.
    // Done once here to avoid repeated division inside the 200 Hz loop.
    _filtAlpha = 1.0f - expf(-2.0f * M_PI * IMU_FILTER_CUTOFF_HZ / LOOP_FREQ_HZ);
    Serial.printf("[IMU] Pitch filter: %.1f Hz cutoff, alpha=%.4f\n",
                  IMU_FILTER_CUTOFF_HZ, _filtAlpha);

    return true;
}

// ============================================================
//  calibrateGyro()
// ============================================================
// Averages GYRO_CAL_SAMPLES stationary readings to determine the
// zero-rate bias. Robot must remain still during this call.
bool IMUSensor::calibrateGyro() {
    Serial.printf("[IMU] Calibrating gyro (%d samples) — keep robot still!\n",
                  GYRO_CAL_SAMPLES);

    float sumX = 0, sumY = 0, sumZ = 0;
    ISM6HG256X_Axes_t g;
    int good = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (_sensor.Get_G_Axes(&g) == ISM6HG256X_OK) {
            sumX += g.x;
            sumY += g.y;
            sumZ += g.z;
            good++;
        }
        delay(2);  // 2 ms per sample = effective 500 Hz sampling.
    }

    if (good < GYRO_CAL_SAMPLES / 2) {
        Serial.println("[IMU] ERROR: too few good readings during calibration");
        return false;
    }

    // Offsets stored in milli-degrees/s to match raw sensor units.
    _gyroOffX = sumX / good;
    _gyroOffY = sumY / good;
    _gyroOffZ = sumZ / good;

    Serial.printf("[IMU] Gyro offsets (mdps): X=%.1f  Y=%.1f  Z=%.1f\n",
                  _gyroOffX, _gyroOffY, _gyroOffZ);
    return true;
}

// ============================================================
//  update() — Sensor read + Mahony AHRS + EMA filter
// ============================================================
void IMUSensor::update(float dt) {
    ISM6HG256X_Axes_t accelRaw, gyroRaw;

    // Abort if I2C fails; retain previous angle to avoid a control spike.
    if (_sensor.Get_X_Axes(&accelRaw) != ISM6HG256X_OK) return;
    if (_sensor.Get_G_Axes(&gyroRaw)  != ISM6HG256X_OK) return;

    // Convert raw milligravity and milli-dps integers to SI-scaled floats.
    _ax = accelRaw.x / 1000.0f;
    _ay = accelRaw.y / 1000.0f;
    _az = accelRaw.z / 1000.0f;

    _gx = (gyroRaw.x - _gyroOffX) / 1000.0f;
    _gy = (gyroRaw.y - _gyroOffY) / 1000.0f;
    _gz = (gyroRaw.z - _gyroOffZ) / 1000.0f;

    // Crude atan2 tilt estimate — valid only when stationary.
    float aPri = axisValue(accelRaw, PITCH_ACCEL_PRIMARY)   / 1000.0f;
    float aSec = axisValue(accelRaw, PITCH_ACCEL_SECONDARY) / 1000.0f;
    _accelAngle = atan2f(aPri, aSec) * RAD_TO_DEG * PITCH_ACCEL_SIGN;

    // Pitch rate on the configured gyro axis.
    float gyroPitch = axisValue(gyroRaw, PITCH_GYRO_AXIS);
    switch (PITCH_GYRO_AXIS) {
        case 'X': gyroPitch -= _gyroOffX; break;
        case 'Y': gyroPitch -= _gyroOffY; break;
        case 'Z': gyroPitch -= _gyroOffZ; break;
    }
    _pitchRate = (gyroPitch / 1000.0f) * PITCH_GYRO_SIGN;

    // Magnetometer reading with hard-iron correction.
    _compass.read();
    _mx = (_compass.getX() - MAG_OFFSET_X) * MAG_SIGN_X;
    _my = (_compass.getY() - MAG_OFFSET_Y) * MAG_SIGN_Y;
    _mz = (_compass.getZ() - MAG_OFFSET_Z) * MAG_SIGN_Z;

    // ---- Mahony AHRS ----
    float ax = _ax, ay = _ay, az = _az;
    float mx = _mx, my = _my, mz = _mz;
    float gx = _gx * DEG_TO_RAD, gy = _gy * DEG_TO_RAD, gz = _gz * DEG_TO_RAD;

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Precompute quaternion products used multiple times below.
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Skip accel correction if I2C returned an all-zero vector (invalid read).
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated gravity direction from current quaternion.
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // Error = cross product of measured vs estimated gravity.
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Vibration rejection: if linear acceleration deviates from 1 G by
        // more than 0.2 G, lower Mahony Kp to trust the gyro over the accel.
        float accelMagGs = sqrtf(_ax*_ax + _ay*_ay + _az*_az);
        float currentKp = MAHONY_KP;
        if (fabsf(accelMagGs - 1.0f) > 0.2f) {
            currentKp = 0.1f;
        }

        // Integral correction for long-term gyro drift.
        if (MAHONY_KI > 0.0f) {
            eInt_x += MAHONY_KI * halfex * dt;
            eInt_y += MAHONY_KI * halfey * dt;
            eInt_z += MAHONY_KI * halfez * dt;
            gx += eInt_x;
            gy += eInt_y;
            gz += eInt_z;
        }

        gx += currentKp * halfex;
        gy += currentKp * halfey;
        gz += currentKp * halfez;
    }

    // Integrate quaternion rate.
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion to unit length.
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;

    // Decompose quaternion to Euler angles.
    float roll_ahrs  = atan2f(q0*q1 + q2*q3, 0.5f - q1q1 - q2q2) * RAD_TO_DEG;
    float pitch_ahrs = asinf(-2.0f * (q1*q3 - q0*q2)) * RAD_TO_DEG;

    // Tilt-compensated yaw: project magnetometer onto the levelled plane
    // so heading is independent of pitch oscillations during balancing.
    float rRad = roll_ahrs * DEG_TO_RAD;
    float pRad = pitch_ahrs * DEG_TO_RAD;
    float Xh = mx * cosf(pRad) + my * sinf(rRad) * sinf(pRad) - mz * cosf(rRad) * sinf(pRad);
    float Yh = my * cosf(rRad) + mz * sinf(rRad);
    _yaw = atan2f(Yh, Xh) * RAD_TO_DEG;

    // Select which AHRS axis maps to balance pitch per config.h mapping.
    if (PITCH_GYRO_AXIS == 'X' || PITCH_GYRO_AXIS == 'x') {
        _pitch = roll_ahrs * (PITCH_GYRO_SIGN * PITCH_ACCEL_SIGN > 0 ? 1 : -1);
    } else if (PITCH_GYRO_AXIS == 'Y' || PITCH_GYRO_AXIS == 'y') {
        _pitch = pitch_ahrs * (PITCH_GYRO_SIGN * PITCH_ACCEL_SIGN > 0 ? 1 : -1);
    } else {
        _pitch = _accelAngle;  // Fallback if axis mapping is misconfigured.
    }

    // 2nd-order cascaded EMA filter (-40 dB/decade rolloff).
    // Seed on first run to avoid initialization transient.
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
//  axisValue()
// ============================================================
// Extracts the float value of a named axis from a raw sensor struct.
float IMUSensor::axisValue(const ISM6HG256X_Axes_t& d, char axis) {
    switch (axis) {
        case 'X': case 'x': return (float)d.x;
        case 'Y': case 'y': return (float)d.y;
        case 'Z': case 'z': return (float)d.z;
        default:             return 0.0f;
    }
}
