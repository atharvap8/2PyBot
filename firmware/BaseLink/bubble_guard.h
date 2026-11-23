#pragma once
#include <math.h>
#include <stdint.h>

// Lidar safety bubble (host computed). Caps the drive reference in BOTH
// directions for EVERY drive source, including a granted navigation lease.
// BG,<en 0|1>,<forward cap m/s>,<reverse cap m/s>
// Enabled guard is fail-closed: no BG for 500 ms caps both directions to 0
// (the robot keeps balancing in place). Steering and balance authority are
// never limited here. BG,0,0,0 releases it.
struct BubbleGuard {
    bool enabled = false;
    uint32_t lastRx = 0;
    float forwardMps = 0.0f;
    float reverseMps = 0.0f;

    bool update(int en, float fwd, float rev, uint32_t now) {
        if ((en != 0 && en != 1) || !isfinite(fwd) || !isfinite(rev) ||
            fwd < 0 || fwd > 3.0f || rev < 0 || rev > 3.0f)
            return false;
        enabled = en != 0;
        forwardMps = fwd;
        reverseMps = rev;
        lastRx = now;
        return true;
    }

    bool fresh(uint32_t now) const { return (uint32_t)(now - lastRx) <= 500; }

    // input and result in m/s (signed, + forward)
    float apply(float input, uint32_t now) const {
        if (!enabled) return input;
        const bool ok = fresh(now);
        if (input > 0.0f) return fminf(input, ok ? forwardMps : 0.0f);
        if (input < 0.0f) return fmaxf(input, ok ? -reverseMps : 0.0f);
        return input;
    }

    bool limiting(float input, uint32_t now) const { return enabled && apply(input, now) != input; }
};
