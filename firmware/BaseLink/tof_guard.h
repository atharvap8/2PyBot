#pragma once
#include <math.h>
#include <stdint.h>

// Limit user forward references, never the balance-recovery wheel acceleration.
// Enabled guard remains fail-closed on link loss; explicit TG,0,0 releases it.
struct ToFGuard {
    bool enabled = false;
    uint32_t lastRx = 0;
    float forwardMps = 0.0f;

    bool update(int en, float limit, uint32_t now) {
        if ((en != 0 && en != 1) || !isfinite(limit) || limit < 0 || limit > 3.0f)
            return false;
        enabled = en != 0;
        forwardMps = limit;
        lastRx = now;
        return true;
    }

    float apply(float input, float fullSpeed, uint32_t now) const {
        if (!enabled || input <= 0.0f) return input;
        const float cap = (uint32_t)(now - lastRx) <= 500 ? forwardMps : 0.0f;
        if (fullSpeed <= 0.0f) return 0.0f;
        return fminf(input, cap / fullSpeed);
    }
};
