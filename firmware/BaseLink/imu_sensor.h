/*
 * ============================================================
 *  imu_sensor.h — ISM6HG256X Wrapper + Complementary Filter
 * ============================================================
 *  Encapsulates the complex initialisation, bias calibration,
 *  and real-time data fusion algorithms required to extract
 *  a mathematically stable pitch and heading from raw IMU data.
 *
 *  Core responsibilities:
 *    - Configure hardware I2C registers for the ISM6HG256X.
 *    - Perform a static zero-rate gyroscope bias calibration.
 *    - Execute a 9-DOF Mahony AHRS (Attitude and Heading 
 *      Reference System) filter.
 *    - Apply a 2nd-order cascaded EMA filter to combat 
 *      high-frequency angular noise.
 * ============================================================
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <ISM6HG256XSensor.h>
#include <QMC5883LCompass.h>
#include "config.h"

// ============================================================
//  IMUSensor Class Definition
// ============================================================
class IMUSensor {
public:
    IMUSensor();

    // Initiates the I2C bus and validates communication with the IMU hardware.
    // Configures the Output Data Rates (ODR) and Full-Scale settings.
    // Returns true if the sensor registers respond correctly, false otherwise.
    bool begin();

    // Aggregates a designated number of readings (GYRO_CAL_SAMPLES) to calculate
    // the persistent static offset of the gyroscope. 
    // The robot must remain perfectly motionless during this execution.
    bool calibrateGyro();

    // The primary mathematical engine for orientation determination.
    // Consumes the time differential (dt) in seconds.
    // Extracts raw sensor vectors, applies calibration offsets, processes the 
    // Mahony AHRS algorithm, and finalises the output through the cascaded filter.
    void update(float dt);

    // --- Processed Output Accessors ---
    
    // Returns the final heavily-filtered pitch angle utilized by the balance PID (degrees).
    float getPitch()     const { return _pitch;      }
    
    // Returns the calculated heading angle tracking relative yaw rotation (degrees).
    float getYaw()       const { return _yaw;        }
    
    // Returns the instantaneous rotational velocity around the pitch axis (degrees/second).
    float getPitchRate() const { return _pitchRate;  }
    
    // Returns the raw, un-fused tilt angle derived exclusively from accelerometer gravity projection.
    float getAccelAngle()const { return _accelAngle; }

    // --- Raw Scaled Data Accessors ---
    // These return the raw vectors localized to standard engineering units.
    float getAx() const { return _ax; } // Accelerometer X (g)
    float getAy() const { return _ay; } // Accelerometer Y (g)
    float getAz() const { return _az; } // Accelerometer Z (g)
    float getGx() const { return _gx; } // Gyroscope X (degrees/second)
    float getGy() const { return _gy; } // Gyroscope Y (degrees/second)
    float getGz() const { return _gz; } // Gyroscope Z (degrees/second)
    float getMx() const { return _mx; } // Magnetometer X (normalized magnitude)
    float getMy() const { return _my; } // Magnetometer Y (normalized magnitude)
    float getMz() const { return _mz; } // Magnetometer Z (normalized magnitude)

private:
    // Underlying hardware abstraction objects.
    ISM6HG256XSensor _sensor;
    QMC5883LCompass _compass;

    // Internal state variables for tracking calculated orientations.
    float _pitch;          // The filtered Mahony AHRS output angle.
    float _yaw;            // The filtered Mahony AHRS heading angle.
    float _pitchRate;      // The isolated rotational velocity constraint.
    float _accelAngle;     // The crude accelerometer tilt calculation.

    // Mahony AHRS integration state variables (quaternions + error integral).
    float q0, q1, q2, q3;
    float eInt_x, eInt_y, eInt_z;

    // Buffer for the current iteration's raw scaled sensor values.
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;

    // Static offsets determined by the calibration routine.
    float _gyroOffX, _gyroOffY, _gyroOffZ;

    // Flag indicating the initial loop iteration, utilized for preventing transient spikes.
    bool _firstReading;

    // Variables managing the 2nd-order cascaded low-pass filter logic.
    float _filtStage1;     // The state variable for the first Exponential Moving Average stage.
    float _filtStage2;     // The state variable for the second sequential EMA stage.
    float _filtAlpha;      // The precomputed smoothing coefficient based on sample frequency.
    bool  _filterSeeded;   // Flag preventing destructive zero-state initialization lag.

    // Utility function extracting a specific dimensional value from the structured hardware payload.
    // Handles axis swapping mathematically without requiring hardware re-mounting.
    static float axisValue(const ISM6HG256X_Axes_t& d, char axis);
};

#endif // IMU_SENSOR_H
