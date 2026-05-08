/*
 * ============================================================
 *  pid_controller.h — Discrete PID Controller
 * ============================================================
 *  Header-only discrete-time PID controller optimized for
 *  high-rate balance robotics.
 *
 *  Features:
 *    - Derivative-on-measurement (eliminates setpoint kick).
 *    - Integral anti-windup clamping.
 *    - First-order low-pass filter on the derivative term.
 *    - Adaptive Kp boost for large-error recovery.
 * ============================================================
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    // Public gains allow live adjustment via the Serial Tuner.
    float Kp;
    float Ki;
    float Kd;
    float setpoint;

    float outputMin;
    float outputMax;
    float integralLimit;

    // Derivative low-pass filter alpha.
    // 1.0 = unfiltered; lower values reduce noise but add lag.
    float dFilterAlpha;

    // Adaptive Kp parameters.
    bool  useAdaptive;
    float adaptiveThreshold;
    float adaptiveBoost;

    // ============================================================
    //  Constructor
    // ============================================================
    PIDController(float kp, float ki, float kd,
                  float sp       = 0.0f,
                  float outMin   = -1e6f,
                  float outMax   =  1e6f,
                  float intLimit =  1e6f,
                  bool adaptive  = false,
                  float adaptThresh = 2.0f,
                  float adaptBoost  = 1.5f,
                  float dAlpha      = 1.0f)
        : Kp(kp), Ki(ki), Kd(kd),
          setpoint(sp),
          outputMin(outMin), outputMax(outMax),
          integralLimit(intLimit),
          dFilterAlpha(dAlpha),
          useAdaptive(adaptive), adaptiveThreshold(adaptThresh), adaptiveBoost(adaptBoost),
          _integral(0.0f), _prevInput(0.0f),
          _lastP(0.0f), _lastI(0.0f), _lastD(0.0f),
          _lastError(0.0f), _firstRun(true) {}

    // ============================================================
    //  compute()
    // ============================================================
    // Executes one control iteration. Must be called at a fixed rate.
    //
    // @param input  Measured process variable (e.g., pitch angle).
    // @param dt     Time elapsed since the last call (seconds).
    // @return       Bounded control output (steps/s for this system).
    // ============================================================
    float compute(float input, float dt) {

        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - input;

        // Adaptive Kp — boost when error exceeds threshold.
        float effKp = Kp;
        if (useAdaptive && fabsf(error) > adaptiveThreshold) {
            effKp = Kp * adaptiveBoost;
        }

        // Proportional term.
        _lastP = effKp * error;

        // Integral term with anti-windup clamp.
        _integral += error * dt;
        _integral  = constrain(_integral, -integralLimit, integralLimit);
        _lastI     = Ki * _integral;

        // Derivative-on-measurement — avoids setpoint-kick on target changes.
        if (_firstRun) {
            _lastD    = 0.0f;
            _firstRun = false;
        } else {
            float dInput = (input - _prevInput) / dt;
            // Negative sign: d(error)/dt = -d(input)/dt when setpoint is constant.
            float rawD = -Kd * dInput;
            _lastD = (dFilterAlpha * rawD) + ((1.0f - dFilterAlpha) * _lastD);
        }

        _prevInput = input;
        _lastError = error;

        float output = _lastP + _lastI + _lastD;
        return constrain(output, outputMin, outputMax);
    }

    // ============================================================
    //  computeAngle()
    // ============================================================
    // Variant of compute() with ±180° error wrapping for continuous
    // rotation targets (e.g., yaw heading control).
    // ============================================================
    float computeAngle(float input, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - input;

        // Wrap error to ±180° for shortest-path rotation.
        while (error >  180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        float effKp = Kp;
        if (useAdaptive && fabsf(error) > adaptiveThreshold) {
            effKp = Kp * adaptiveBoost;
        }

        _lastP = effKp * error;

        _integral += error * dt;
        _integral  = constrain(_integral, -integralLimit, integralLimit);
        _lastI     = Ki * _integral;

        if (_firstRun) {
            _lastD    = 0.0f;
            _firstRun = false;
        } else {
            float dInput = input - _prevInput;
            // Wrap derivative input to handle 180°/-180° boundary crossings.
            while (dInput >  180.0f) dInput -= 360.0f;
            while (dInput < -180.0f) dInput += 360.0f;

            float rawD = -Kd * (dInput / dt);
            _lastD = (dFilterAlpha * rawD) + ((1.0f - dFilterAlpha) * _lastD);
        }

        _prevInput = input;
        _lastError = error;

        float output = _lastP + _lastI + _lastD;
        return constrain(output, outputMin, outputMax);
    }

    // ============================================================
    //  reset()
    // ============================================================
    // Clears all internal state. Call before enabling motors to
    // prevent integral wind-up carry-over from a previous session.
    void reset() {
        _integral  = 0.0f;
        _prevInput = 0.0f;
        _lastP     = 0.0f;
        _lastI     = 0.0f;
        _lastD     = 0.0f;
        _lastError = 0.0f;
        _firstRun  = true;
    }

    // Telemetry accessors — read-only, non-invasive.
    float getP()     const { return _lastP;     }
    float getI()     const { return _lastI;     }
    float getD()     const { return _lastD;     }
    float getError() const { return _lastError; }

private:
    float _integral;
    float _prevInput;
    float _lastP, _lastI, _lastD;
    float _lastError;
    bool  _firstRun;
};

#endif // PID_CONTROLLER_H
