#pragma once
#include <cmath>

// Robot-forward units throughout. Wiring polarity belongs only at the UART boundary.
namespace hub_control {
inline float clip(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

struct Settings {
    float kp, ki, integralLimit, feedForward, mass, radius, torqueConstant;
    float yawKp, yawKi, yawIntegralLimit, yawCurrentSlew, currentLimit;
};

struct Currents { float left = 0, right = 0; bool limited = false; };

struct VelocityCommand { float value; bool trackingLimited; };

// Back-calculate only the virtual actuator state, in units of available
// current / velocity gain. This is not an absolute speed or stick-input cap:
// the same tracking error is allowed at 0.1, 1.0 or 1.5 m/s.
inline VelocityCommand integrateVelocity(float previous, float acceleration, float measured,
                                         float dt, float kp, float currentLimit, float explicitSpeedLimit) {
    const float requested = previous + acceleration * dt;
    float value = requested;
    if (kp > .0001f) {
        const float lead = currentLimit / kp;
        value = clip(value, measured - lead, measured + lead);
    }
    if (explicitSpeedLimit > 0) value = clip(value, -explicitSpeedLimit, explicitSpeedLimit);
    return {value, std::fabs(value-requested) > .00001f};
}

class CurrentLoop {
public:
    float commonIntegral = 0, yawIntegral = 0, yawCurrent = 0;
    float feedForwardCurrent = 0, velocityCorrection = 0;
    bool recoveryPriority = false;

    void reset(float commonTarget = 0) {
        commonIntegral = yawIntegral = yawCurrent = 0;
        feedForwardCurrent = velocityCorrection = 0;
        recoveryPriority = false;
        previousTarget = commonTarget;
    }

    Currents update(float leftTarget, float rightTarget, float leftVelocity,
                    float rightVelocity, float dt, const Settings& s,
                    float balanceAcceleration = NAN, bool recovering = false) {
        Currents out;
        if (!(dt > 0 && dt <= .05f) || !std::isfinite(leftTarget + rightTarget + leftVelocity + rightVelocity)) {
            reset();
            return out;
        }
        const float target = .5f * (leftTarget + rightTarget);
        const float error = target - .5f * (leftVelocity + rightVelocity);
        // Balance supplies its acceleration explicitly: a clipped or
        // back-calculated speed target must not erase a catch command.
        // Target differentiation remains only for the disarmed bench routine.
        const float accel = std::isfinite(balanceAcceleration) ? balanceAcceleration : (target - previousTarget) / dt;
        previousTarget = target;
        // Feed-forward is COMMON acceleration only. A turn must not inject
        // opposing full-mass acceleration impulses into the two wheels.
        const float ff = s.feedForward * s.mass * s.radius / (2 * s.torqueConstant) * accel;
        float correction = s.kp * error + commonIntegral;
        recoveryPriority = recovering && accel * correction < 0;
        if (recoveryPriority) {
            // Do not brake against the balance controller just because the
            // wheels outran a virtual velocity target during a forward fall.
            correction = 0;
            commonIntegral *= 1 - clip(8 * dt, 0, 1);
        }
        const float commonPre = ff + correction;
        if (!recoveryPriority && !((commonPre >= s.currentLimit && error > 0) || (commonPre <= -s.currentLimit && error < 0)))
            commonIntegral = clip(commonIntegral + s.ki * error * dt, -s.integralLimit, s.integralLimit);
        correction = recoveryPriority ? 0 : s.kp * error + commonIntegral;
        const float common = clip(ff + correction, -s.currentLimit, s.currentLimit);
        feedForwardCurrent = ff;
        velocityCorrection = correction;
        const float room = std::fmax(0.0f, s.currentLimit - std::fabs(common));
        const float yawError = .5f * ((rightTarget - leftTarget) - (rightVelocity - leftVelocity));
        const float yawPre = s.yawKp * yawError + yawIntegral;
        const float wanted = clip(yawPre, -room, room);
        const float slewed = yawCurrent + clip(wanted - yawCurrent, -s.yawCurrentSlew * dt, s.yawCurrentSlew * dt);
        // Track both saturation and slew limiting: no stored turn kick after a shove.
        if (!((yawPre > slewed && yawError > 0) || (yawPre < slewed && yawError < 0)))
            yawIntegral = clip(yawIntegral + s.yawKi * yawError * dt, -s.yawIntegralLimit, s.yawIntegralLimit);
        yawCurrent = clip(slewed, -room, room);
        out.left = common - yawCurrent;
        out.right = common + yawCurrent;
        out.limited = std::fabs(ff + correction) > s.currentLimit || std::fabs(yawPre) > room;
        return out;
    }

private:
    float previousTarget = 0;
};

struct Steering {
    float command = 0, heading = 0;
    bool releasing = false;

    void reset(float differential) { command = 0; heading = differential; releasing = false; }

    float update(float target, bool turning, float differential, float differentialRate,
                 float look, float kp, float kd, float holdLimit, float relatch,
                 bool suspend, float slew, float room, float settleRate, float dt) {
        if (turning) {
            heading = differential;
            releasing = true;
        } else if (releasing && std::fabs(look) < .01f) {
            // Brake to zero wheel differential first, then latch the final
            // heading. Do not immediately fight the turn's residual motion.
            heading = differential;
            target = 0;
            if (std::fabs(command) < 1 && std::fabs(differentialRate) < settleRate) releasing = false;
        } else {
            releasing = false;
            float error = heading + look - differential;
            if (std::fabs(error) > relatch || suspend) { heading = differential - look; error = 0; }
            target = suspend ? 0 : clip(kp * error - kd * differentialRate, -holdLimit, holdLimit);
        }
        target = clip(target, -room, room);
        command += clip(target - command, -slew * dt, slew * dt);
        command = clip(command, -room, room);
        return command;
    }
};
} // namespace hub_control
