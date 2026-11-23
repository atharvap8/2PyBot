#pragma once
#include <math.h>
#include <stdint.h>

// Lidar safety bubble, v2 (4.7.0). The Radxa sends free DISTANCES, not speeds:
//   BG,<en 0|1>,<front_m>,<back_m>,<flags>,<age_ms>
// front/back: free travel (m) from the body to the nearest return inside the
// swept corridor of the current path (straight or arc). flags bit0: turning
// left (CCW) would hit, bit1: turning right would hit. age_ms: how old the
// scan was when the line was sent.
//
// The controller then works out the safe speed EVERY control cycle:
//   1. clearance now = sent clearance - (travel since the scan, from the wheels)
//   2. cap = braking speed for (clearance - margin) with decel and latency
// so a fast approach is limited by distance actually left, not by a cap that
// was computed 100+ ms earlier. Every drive source is limited, including the
// navigation lease and auto-climb. Steering and balance recovery are not
// limited except that a commanded turn into an obstacle is blocked.
// Fail-closed: an enabled bubble without a fresh BG for staleMs gives zero
// clearance (the robot holds position while balancing). BG,0,... releases it.
struct BubbleGuard {
    bool enabled = false;
    uint32_t lastRx = 0;
    float front0 = 0, back0 = 0;   // clearance at posAtRx, already corrected for scan age
    float posAtRx = 0;
    uint8_t flags = 0;
    bool limitedNow = false;
    float fwdCapNow = 0, revCapNow = 0, frontNow = 0, backNow = 0;

    bool update(int en, float front, float back, int fl, int ageMs, uint32_t now, float posM, float velF) {
        if ((en != 0 && en != 1) || !isfinite(front) || !isfinite(back) || front < 0 || back < 0 ||
            front > 20.0f || back > 20.0f || fl < 0 || fl > 3 || ageMs < 0 || ageMs > 1000)
            return false;
        enabled = en != 0;
        const float age = ageMs * 0.001f;
        // Distance already driven since the scan was taken (wheel speed * age).
        front0 = front - fmaxf(0.0f, velF) * age;
        back0  = back  - fmaxf(0.0f, -velF) * age;
        posAtRx = posM; flags = (uint8_t)fl; lastRx = now;
        return true;
    }

    bool fresh(uint32_t now, float staleMs) const { return (uint32_t)(now - lastRx) <= (uint32_t)staleMs; }

    static float brakingCap(float clearance, float margin, float decel, float latency) {
        const float c = clearance - margin;
        if (c <= 0.0f || decel <= 0.0f) return 0.0f;
        const float dl = decel * latency;
        return sqrtf(dl * dl + 2.0f * decel * c) - dl;
    }

    // input/result: drive reference in m/s (+ forward)
    float apply(float input, uint32_t now, float posM, float margin, float decel, float latency, float staleMs) {
        if (!enabled) { limitedNow = false; return input; }
        const bool ok = fresh(now, staleMs);
        const float moved = posM - posAtRx;
        frontNow = ok ? front0 - moved : 0.0f;
        backNow  = ok ? back0 + moved : 0.0f;
        fwdCapNow = brakingCap(frontNow, margin, decel, latency);
        revCapNow = brakingCap(backNow, margin, decel, latency);
        float out = input;
        if (input > fwdCapNow) out = fwdCapNow;
        if (input < -revCapNow) out = -revCapNow;
        limitedNow = out != input;
        return out;
    }

    // steer > 0 = CCW (left), < 0 = CW (right). Blocked directions give 0.
    float limitSteer(float steer, uint32_t now, float staleMs) const {
        if (!enabled) return steer;
        if (!fresh(now, staleMs)) return steer;          // stale: speed is already 0; allow turning away
        if (steer > 0 && (flags & 1)) return 0.0f;
        if (steer < 0 && (flags & 2)) return 0.0f;
        return steer;
    }
};
