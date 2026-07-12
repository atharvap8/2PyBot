/*
 * ============================================================
 *  fall_protection.h — Predictive Fall Protection
 * ============================================================
 *  Estimates imminent-fall risk before the hard MAX_TILT_ANGLE
 *  cutoff is reached, and de-rates operator inputs so the
 *  balance PID keeps full wheel authority for recovery.
 *
 *  Strategy (soft intervention, NOT a kill switch):
 *    1. Predict pitch a short horizon ahead:
 *         predicted = pitch + pitchRate * FP_PREDICT_HORIZON_S
 *    2. Map |predicted| between FP_RISK_START_DEG and
 *       FP_RISK_FULL_DEG into a risk factor 0..1.
 *    3. While at risk:
 *         - Operator lean commands are scaled down / cancelled
 *           so the robot stops accelerating toward the fall.
 *         - Steering is constrained (a differential turn steals
 *           step-rate authority from the balance loop and adds
 *           centripetal disturbance mid-turn).
 *    4. Hysteresis + hold time prevent rapid toggling.
 *
 *  The balance PID itself is never limited — catching a fall
 *  requires driving the wheels toward it at full authority.
 * ============================================================
 */

#ifndef FALL_PROTECTION_H
#define FALL_PROTECTION_H

#include <Arduino.h>
#include "config.h"

class FallProtection {
public:
    bool  enabled = true;

    // Tunable at runtime via serial config.
    float predictHorizonS = FP_PREDICT_HORIZON_S;
    float riskStartDeg    = FP_RISK_START_DEG;
    float riskFullDeg     = FP_RISK_FULL_DEG;

    // ------------------------------------------------------
    //  update() — call once per control loop, before using
    //  the constrain*() helpers.
    // ------------------------------------------------------
    //  pitch      : current pitch relative to equilibrium (deg)
    //  pitchRate  : angular velocity (deg/s)
    // ------------------------------------------------------
    void update(float pitch, float pitchRate, float dt) {
        if (!enabled) {
            risk_ = 0.0f;
            active_ = false;
            return;
        }

        predicted_ = pitch + pitchRate * predictHorizonS;

        // Risk rises when the robot is moving TOWARD the predicted
        // fall (pitch and pitchRate same sign) — a fast return to
        // center is recovery, not danger.
        float mag = fabsf(predicted_);
        bool movingTowardFall = (pitch * pitchRate) > 0.0f;

        float targetRisk = 0.0f;
        if (mag > riskStartDeg && (movingTowardFall || fabsf(pitch) > riskStartDeg)) {
            targetRisk = (mag - riskStartDeg) / (riskFullDeg - riskStartDeg);
            targetRisk = constrain(targetRisk, 0.0f, 1.0f);
        }

        // Fast attack, slow release: react within one loop tick,
        // relax over ~FP_RELEASE_TIME_S to avoid oscillation.
        if (targetRisk > risk_) {
            risk_ = targetRisk;
        } else {
            float releaseAlpha = 1.0f - expf(-dt / FP_RELEASE_TIME_S);
            risk_ += (targetRisk - risk_) * releaseAlpha;
        }

        // Hysteresis on the active flag.
        if (!active_ && risk_ > FP_ACTIVE_ON_THRESHOLD)  active_ = true;
        if (active_  && risk_ < FP_ACTIVE_OFF_THRESHOLD) active_ = false;
    }

    // ------------------------------------------------------
    //  Input constraints (apply to OPERATOR inputs only)
    // ------------------------------------------------------

    // Scales a commanded lean offset (deg). Additionally, any lean
    // command pointing INTO the predicted fall is cut entirely —
    // that is the "running toward its fall" case.
    float constrainDrive(float leanCmd) const {
        if (!active_) return leanCmd;
        bool intoFall = (leanCmd * predicted_) > 0.0f;
        if (intoFall) return 0.0f;
        return leanCmd * (1.0f - risk_);
    }

    // Scales a steering command (deg/s or normalized). During risk
    // the turn rate is constrained so differential stepping cannot
    // consume wheel authority needed for the balance recovery.
    float constrainSteering(float steerCmd) const {
        if (!active_) return steerCmd;
        float k = 1.0f - risk_ * FP_STEER_CUT_FACTOR;
        return steerCmd * constrain(k, 0.0f, 1.0f);
    }

    // Scales the differential (yaw) wheel-speed split at the output
    // stage — the final guard for turns already in progress.
    float constrainYawOutput(float yawOut) const {
        if (!active_) return yawOut;
        float k = 1.0f - risk_ * FP_STEER_CUT_FACTOR;
        return yawOut * constrain(k, 0.0f, 1.0f);
    }

    bool  isActive()  const { return active_; }
    float getRisk()   const { return risk_; }
    float getPredictedPitch() const { return predicted_; }

private:
    float risk_      = 0.0f;   // 0 = safe, 1 = imminent fall
    float predicted_ = 0.0f;   // extrapolated pitch (deg)
    bool  active_    = false;
};

#endif // FALL_PROTECTION_H
