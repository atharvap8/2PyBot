/*
 * ============================================================
 *  imu_sensor.h — ISM6HG256X + QMC5883L Wrapper
 * ============================================================
 *  Encapsulates I2C initialization, gyro bias calibration,
 *  and real-time 9-DOF Mahony AHRS fusion to produce a
 *  stable pitch angle and compass heading.
 *
 *  Responsibilities:
 *    - Configure ISM6HG256X ODR and full-scale ranges.
 *    - Perform static zero-rate gyro bias calibration.
 *    - Run the Mahony AHRS filter each loop.
 *    - Apply a 2nd-order cascaded EMA filter to pitch output.
 * ============================================================
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <ISM6HG256XSensor.h>
#include <QMC5883LCompass.h>
#include "config.h"

class IMUSensor {
public:
    IMUSensor();

    // Initializes I2C, validates IMU presence, configures ODR and
    // full-scale, inits the magnetometer, and precomputes the filter
    // coefficient. Returns false if any hardware check fails.
    bool begin();

    // Collects GYRO_CAL_SAMPLES readings at rest to compute the
    // static gyro bias offsets. Robot must be stationary.
    bool calibrateGyro();

    // Reads raw sensor data, runs Mahony AHRS, applies the cascaded
    // EMA filter, and updates all output values. Call once per loop.
    void update(float dt);

    // --- Processed output accessors ---
    float getPitch()     const { return _pitch;      }  // Filtered pitch angle (degrees).
    float getYaw()       const { return _yaw;        }  // Compass heading (degrees).
    float getPitchRate() const { return _pitchRate;  }  // Pitch angular velocity (°/s).
    float getAccelAngle()const { return _accelAngle; }  // Raw accelerometer tilt estimate.

    // --- Raw scaled sensor accessors ---
    float getAx() const { return _ax; }  // Accelerometer X (g)
    float getAy() const { return _ay; }  // Accelerometer Y (g)
    float getAz() const { return _az; }  // Accelerometer Z (g)
    float getGx() const { return _gx; }  // Gyroscope X (°/s)
    float getGy() const { return _gy; }  // Gyroscope Y (°/s)
    float getGz() const { return _gz; }  // Gyroscope Z (°/s)
    float getMx() const { return _mx; }  // Magnetometer X (normalized)
    float getMy() const { return _my; }  // Magnetometer Y (normalized)
    float getMz() const { return _mz; }  // Magnetometer Z (normalized)

private:
    ISM6HG256XSensor _sensor;
    QMC5883LCompass  _compass;

    float _pitch, _yaw, _pitchRate, _accelAngle;

    // Mahony AHRS quaternion state.
    float q0, q1, q2, q3;
    float eInt_x, eInt_y, eInt_z;

    // Scaled sensor values for the current iteration.
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;

    // Static gyro bias offsets from calibration.
    float _gyroOffX, _gyroOffY, _gyroOffZ;

    bool _firstReading;

    // 2nd-order cascaded EMA filter state.
    float _filtStage1;
    float _filtStage2;
    float _filtAlpha;
    bool  _filterSeeded;

    // Returns the float value of the requested axis from a raw sensor struct.
    static float axisValue(const ISM6HG256X_Axes_t& d, char axis);
};

#endif // IMU_SENSOR_H
