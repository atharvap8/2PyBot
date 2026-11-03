#pragma once
#include <cmath>
#include <cstdint>

// Front-bumper experiment: forward wheel torque reacts against forward lean.
// Only a local START press calls start(); there are no autonomous retries.
class SelfRighting {
public:
    enum Phase : uint8_t { IDLE, SETTLE, LIFT, CAPTURE, HANDED_OFF, ABORTED };
    enum Reason : uint8_t {
        NONE, OPERATOR, NOT_AT_REST, TIMEOUT, NO_LIFT, WRONG_WAY,
        ANGLE_LIMIT, SPEED_LIMIT, TRAVEL_LIMIT, INVALID_SAMPLE, RELAPSE,
        MOTOR_FAULT, PAD_LOST, IMU_STALE, STICK_MOVED, RATE_LIMIT, HOST_LOST
    };
    struct Settings {
        float restDeg, currentAmps, kp, kd, currentSlew, timeout, maxTravel, maxSpeed;
    };
    Phase phase = IDLE;
    Reason reason = NONE;
    uint32_t attempt = 0;
    float elapsed = 0, currentAmps = 0, minimumPitch = 0, travelL = 0, travelR = 0;

    bool active() const { return phase == SETTLE || phase == LIFT || phase == CAPTURE; }

    bool start(const Settings& settings, float pitch, float rate, float leftVelocity,
               float rightVelocity, float leftPosition, float rightPosition, uint32_t now) {
        if (active()) return false;
        if (!std::isfinite(pitch + rate + leftVelocity + rightVelocity + leftPosition + rightPosition) ||
            std::fabs(pitch - settings.restDeg) > 10 || std::fabs(rate) > 15 ||
            std::fabs(leftVelocity) > .08f || std::fabs(rightVelocity) > .08f)
            return false;
        tuning = settings;
        phase = SETTLE; reason = NONE; ++attempt;
        startedMs = now; elapsed = currentAmps = travelL = travelR = 0;
        startPitch = minimumPitch = pitch;
        startL = leftPosition; startR = rightPosition;
        return true;
    }

    void abort(Reason why) {
        if (!active()) return;
        phase = ABORTED; reason = why; currentAmps = 0;
    }

    void update(float pitch, float rate, float leftVelocity, float rightVelocity,
                float leftPosition, float rightPosition, float dt, uint32_t now) {
        if (!active()) return;
        if (!(dt > 0 && dt <= .02f) ||
            !std::isfinite(pitch + rate + leftVelocity + rightVelocity + leftPosition + rightPosition)) {
            abort(INVALID_SAMPLE); return;
        }
        elapsed = (now - startedMs) * .001f;
        minimumPitch = std::fmin(minimumPitch, pitch);
        travelL = leftPosition - startL; travelR = rightPosition - startR;
        if (elapsed > tuning.timeout) { abort(TIMEOUT); return; }
        if (pitch > 75 || pitch < -12) { abort(ANGLE_LIMIT); return; }
        if (std::fabs(rate) > 250) { abort(RATE_LIMIT); return; }
        if (std::fabs(leftVelocity) > tuning.maxSpeed || std::fabs(rightVelocity) > tuning.maxSpeed) {
            abort(SPEED_LIMIT); return;
        }
        if (std::fabs(travelL) > tuning.maxTravel || std::fabs(travelR) > tuning.maxTravel) {
            abort(TRAVEL_LIMIT); return;
        }
        if (phase == SETTLE) {
            if (std::fabs(pitch - tuning.restDeg) > 10 || std::fabs(rate) > 15 ||
                std::fabs(leftVelocity) > .08f || std::fabs(rightVelocity) > .08f) {
                abort(NOT_AT_REST); return;
            }
            if (elapsed < .20f) return;
            phase = LIFT;
        }
        if (pitch > startPitch + 5) { abort(WRONG_WAY); return; }
        if (elapsed > .75f && minimumPitch > startPitch - 3) { abort(NO_LIFT); return; }
        if (phase == CAPTURE && minimumPitch < 25 && pitch > 35) { abort(RELAPSE); return; }
        if (pitch < 25 || pitch + rate * .08f < 25) phase = CAPTURE;
        if (std::fabs(pitch) <= 8 && std::fabs(rate) <= 70 && std::fabs(pitch + rate * .06f) <= 8) {
            phase = HANDED_OFF;
            return;
        }
        // Ground-position/velocity LQI terms are not valid on the bumper.
        // Negative pitch rate reduces/reverses current to catch the rise.
        const float wanted = clip(tuning.kp * pitch + tuning.kd * rate,
                                  -tuning.currentAmps, tuning.currentAmps);
        const float step = tuning.currentSlew * dt;
        currentAmps = clip(currentAmps + clip(wanted - currentAmps, -step, step),
                           -tuning.currentAmps, tuning.currentAmps);
    }

    static const char* reasonName(Reason why) {
        switch (why) {
            case NONE: return "none";
            case OPERATOR: return "operator stop";
            case NOT_AT_REST: return "front rest moved during settle";
            case TIMEOUT: return "attempt timeout";
            case NO_LIFT: return "less than 3 deg lift after 0.75 s";
            case WRONG_WAY: return "lean increased by 5 deg";
            case ANGLE_LIMIT: return "outside recovery angle envelope";
            case SPEED_LIMIT: return "wheel speed limit";
            case TRAVEL_LIMIT: return "wheel travel limit";
            case INVALID_SAMPLE: return "invalid sample/control interval";
            case RELAPSE: return "fell forward during capture";
            case MOTOR_FAULT: return "motor link/mode/fault";
            case PAD_LOST: return "local gamepad disconnected/stale";
            case IMU_STALE: return "IMU samples stale";
            case STICK_MOVED: return "drive stick/host command moved";
            case RATE_LIMIT: return "pitch rate limit";
            case HOST_LOST: return "host readiness lost";
        }
        return "unknown";
    }

private:
    Settings tuning{};
    uint32_t startedMs = 0;
    float startPitch = 0, startL = 0, startR = 0;
    static float clip(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
};
