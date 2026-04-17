/*
 * ============================================================
 *  pid_controller.h — Discrete PID Controller
 * ============================================================
 *  Header-only implementation of a discrete-time Proportional,
 *  Integral, and Derivative controller optimized for high-speed
 *  balance robotics.
 *
 *  Core Features:
 *    - Derivative-on-measurement formulation eliminates massive 
 *      setpoint-kick spikes.
 *    - Integral anti-windup clamping prevents uncontrolled 
 *      accumulation during physical stalls.
 *    - Internal 1st-order low-pass filtering on the derivative term 
 *      mitigates high-frequency IMU quantization noise.
 *    - Adaptive proportional gain seamlessly boosts responsiveness
 *      when recovering from severe, unexpected angular impacts.
 * ============================================================
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    // --- Functional Tuning Constants ---
    // These variables deliberately remain fully public allowing 
    // seamless, on-the-fly adjustment via the external Serial Tuner thread.
    float Kp;           // Proportional gain multiplier.
    float Ki;           // Integral gain multiplier.
    float Kd;           // Derivative gain multiplier.
    float setpoint;     // The algorithmic target state (typically 0.0 degrees).

    // --- Output Saturation Constraints ---
    float outputMin;    // Fundamental floor constraint placed natively on the computed output.
    float outputMax;    // Fundamental ceiling constraint placed natively on the computed output.
    float integralLimit;// Absolute absolute constraint placed squarely on the time-dependent integral accumulator.
    
    // --- First-Order Derivative Low-Pass Filter ---
    // Mathematically isolates the derivative term from sudden, meaningless 
    // single-tick signal variations.
    // 1.0 represents an unfiltered, raw transmission.
    // 0.1 represents a heavily filtered, latency-inducing transmission.
    float dFilterAlpha;
    
    // --- Adaptive Proportional Reaction Physics ---
    bool useAdaptive;           // Toggles the dynamic Kp multiplication algorithm.
    float adaptiveThreshold;    // The absolute error degree at which the multiplier physically engages.
    float adaptiveBoost;        // The explicit scalar applied linearly against Kp during extreme recoveries.

    // ============================================================
    //  Constructor
    // ============================================================
    // Ingress parameters structure the logical foundation of the controller.
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
    // Orchestrates processing of a single chronological control iteration.
    // Must be invoked synchronously with guaranteed periodicity (e.g., exactly 200 Hz).
    //
    // @param input   The measured process variable currently manifesting in physics.
    // @param dt      The precisely quantified time elapsed since the previous invocation (seconds).
    // @return        The computed, bounded reactionary output value directing the hardware.
    // ============================================================
    float compute(float input, float dt) {
        
        // Abandon computation entirely if mathematical time has inexplicably frozen or reversed.
        if (dt <= 0.0f) return 0.0f;

        // Establish the fundamental baseline error delta.
        float error = setpoint - input;

        // --- Adaptive Gain Evaluation ---
        // Dynamically override the proportional multiplier temporarily if the 
        // measured mechanical error vastly exceeds structural tolerance norms.
        float effKp = Kp;
        if (useAdaptive && fabsf(error) > adaptiveThreshold) {
            effKp = Kp * adaptiveBoost;
        }

        // --- Proportional Calculation ---
        // Generates an immediate linear force strictly relative to current discrepancy.
        _lastP = effKp * error;

        // --- Integral Calculation ---
        // Accumulates prolonged, tiny static discrepancies mathematically over time.
        // The embedded 'anti-windup' limit truncates the sum, specifically protecting 
        // the chassis from catastrophic failure if physically jammed against an obstacle.
        _integral += error * dt;
        _integral  = constrain(_integral, -integralLimit, integralLimit);
        _lastI     = Ki * _integral;

        // --- Derivative Calculation (Derivative-on-Measurement) ---
        if (_firstRun) {
            // Nullify derivative initialization spikes that manifest falsely 
            // when delta-time mathematics are established for the very first reading.
            _lastD    = 0.0f;
            _firstRun = false;
        } else {
            // By measuring the rate of change sequentially across the INPUT itself
            // rather than the ERROR, we entirely circumvent catastrophic 'Setpoint Kick'.
            // This prevents the robot from jerking violently if the user suddenly commands
            // a new target pitch via the joystick.
            float dInput = (input - _prevInput) / dt;
            
            // The negative sign is requisite because d(Error) would inherently be 
            // inverted relative to d(Input).
            float rawD = -Kd * dInput;
            
            // Implement Exponential Moving Average smoothing specifically localized 
            // exclusively isolating the derivative spectrum.
            _lastD = (dFilterAlpha * rawD) + ((1.0f - dFilterAlpha) * _lastD);
        }
        
        // Commit tracking variables permanently into history.
        _prevInput = input;
        _lastError = error;

        // --- Aggregation ---
        // Synthesize the localized mathematical constructs into unified, driving kinetic energy.
        float output = _lastP + _lastI + _lastD;
        output = constrain(output, outputMin, outputMax);

        return output;
    }

    // ============================================================
    //  computeAngle()
    // ============================================================
    // A structurally identical variant of the compute algorithm outfitted with 
    // explicit topological wrapping logic. 
    // Executed universally when manipulating continuous 360-degree rotational entities
    // to strictly enforce 'shortest-path' navigational decisions.
    // ============================================================
    float computeAngle(float input, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - input;

        // Topologically bound mathematical error inherently to a ±180-degree manifold.
        // Prevents the chassis from absurdly attempting a 359-degree right turn
        // to accomplish a seemingly simple 1-degree left correction.
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        // --- Adaptive Gain Evaluation ---
        float effKp = Kp;
        if (useAdaptive && fabsf(error) > adaptiveThreshold) {
            effKp = Kp * adaptiveBoost;
        }

        // --- Proportional Calculation ---
        _lastP = effKp * error;

        // --- Integral Calculation ---
        _integral += error * dt;
        _integral  = constrain(_integral, -integralLimit, integralLimit);
        _lastI     = Ki * _integral;

        // --- Derivative Calculation (Derivative-on-Measurement) ---
        if (_firstRun) {
            _lastD    = 0.0f;
            _firstRun = false;
        } else {
            float dInput = input - _prevInput;
            
            // Re-apply topological wrapping precisely localized immediately onto the 
            // differential measurement natively. This intercepts massive derivative 
            // explosions occurring exactly at the moment the input mathematically wraps past ±180.
            while (dInput > 180.0f) dInput -= 360.0f;
            while (dInput < -180.0f) dInput += 360.0f;

            float rawD = -Kd * (dInput / dt);
            _lastD = (dFilterAlpha * rawD) + ((1.0f - dFilterAlpha) * _lastD);
        }
        _prevInput = input;
        _lastError = error;

        // --- Aggregation ---
        float output = _lastP + _lastI + _lastD;
        output = constrain(output, outputMin, outputMax);

        return output;
    }

    // ============================================================
    //  reset()
    // ============================================================
    // Hard-clears all sequential internal mathematics.
    // Strictly mandated whenever engaging motors transitioning from an idle state.
    void reset() {
        _integral  = 0.0f;
        _prevInput = 0.0f;
        _lastP     = 0.0f;
        _lastI     = 0.0f;
        _lastD     = 0.0f;
        _lastError = 0.0f;
        _firstRun  = true;
    }

    // --- Telemetry Getters ---
    // Read-only functions servicing non-invasive data serialization via external debug interfaces.
    float getP()     const { return _lastP;     }
    float getI()     const { return _lastI;     }
    float getD()     const { return _lastD;     }
    float getError() const { return _lastError; }

private:
    float _integral;          // Persistent summation of accumulated discrepancies.
    float _prevInput;         // Chronological reference utilized for determining derivative variance.
    float _lastP, _lastI, _lastD; // Historical records representing individual isolated outputs.
    float _lastError;         // Mathematical delta of the preceding iteration.
    bool  _firstRun;          // Toggles logic preventing explosive initialization discrepancies.
};

#endif // PID_CONTROLLER_H
