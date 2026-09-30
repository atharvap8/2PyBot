/*
 * ============================================================
 *  terrain.h — road-condition awareness and adaptive gains
 * ============================================================
 *  Three jobs, all cheap enough to run every control loop:
 *
 *  1. ROUGHNESS  a slow EMA of how far |accel| strays from 1 g.
 *     Smooth floor ~0.02, a bad footpath ~0.15, a kerb strike spikes
 *     past 0.5. Used to SOFTEN the high-frequency gains continuously:
 *     on broken ground a stiff K3/K4 fights every bump and the robot
 *     chatters, so we back them off in proportion to the roughness.
 *
 *  2. DROP       free-fall detection. |accel| below DROPG for DROPMS
 *     means a wheel (or the whole robot) is airborne — coming off a
 *     doorstep or a kerb. While airborne the wheels have NO authority,
 *     so the integrator is frozen (otherwise it winds up against an
 *     error it cannot fix and the robot lunges on touchdown).
 *
 *  3. LANDING    for LANDMS after the impact, the soft gain set is
 *     used outright. The wheel arrives with a large transient and the
 *     right response is to yield, not to fight it.
 *
 *  Everything is bounded: softening never exceeds TERRSOFT, and the
 *  whole module is disabled by setting TERREN to 0.
 * ============================================================
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include <Arduino.h>
#include "params.h"

enum TerrState : uint8_t { TERR_FLAT = 0, TERR_ROUGH = 1, TERR_AIR = 2, TERR_LAND = 3 };

struct Terrain {
    float    rough     = 0.0f;   // 0 = glass, >0.3 = bad ground
    float    accelMag  = 1.0f;   // g
    float    soften    = 1.0f;   // gain multiplier, TERRSOFT..1
    uint8_t  state     = TERR_FLAT;
    uint32_t airStart  = 0;
    uint32_t landUntil = 0;
    uint32_t drops     = 0;      // how many step-downs this session
    float    peakG     = 0.0f;   // worst impact seen, for the log
};

static Terrain TERR;

/*
 * Call once per control loop with the raw accelerometer magnitude in g
 * and the measured pitch rate. Returns nothing; read TERR.
 */
inline void terrain_update(float accelMagG, float pitchRateDps, float dt) {
    TERR.accelMag = accelMagG;
    uint32_t now = millis();

    if (P.terrEnable < 0.5f) {
        TERR.rough = 0.0f; TERR.soften = 1.0f; TERR.state = TERR_FLAT;
        return;
    }

    // ---- 1. roughness: EMA of |1g - measured|, plus a rate-spike term ----
    float dev  = fabsf(accelMagG - 1.0f);
    float spin = fabsf(pitchRateDps) / 400.0f;            // 400 deg/s == 1.0
    float inst = dev + 0.35f * spin;
    float a    = dt / (P.terrTau + dt);                   // first-order lag
    TERR.rough += a * (inst - TERR.rough);
    if (dev > TERR.peakG) TERR.peakG = dev;

    // ---- 2. free fall ----
    if (accelMagG < P.dropG) {
        if (TERR.airStart == 0) TERR.airStart = now;
        if (now - TERR.airStart >= (uint32_t)P.dropMs) {
            if (TERR.state != TERR_AIR) TERR.drops++;
            TERR.state = TERR_AIR;
        }
    } else {
        if (TERR.state == TERR_AIR) {                     // touchdown
            TERR.landUntil = now + (uint32_t)P.landMs;
            TERR.state = TERR_LAND;
        }
        TERR.airStart = 0;
    }

    if (TERR.state == TERR_LAND && now > TERR.landUntil) TERR.state = TERR_FLAT;

    // ---- 3. gain multiplier ----
    if (TERR.state == TERR_AIR || TERR.state == TERR_LAND) {
        TERR.soften = P.terrSoft;                         // full softening
    } else {
        // linear from 1.0 at rough=0 down to TERRSOFT at rough=TERRTHR
        float t = TERR.rough / (P.terrThr > 0.01f ? P.terrThr : 0.01f);
        if (t > 1.0f) t = 1.0f;
        TERR.soften = 1.0f - t * (1.0f - P.terrSoft);
        TERR.state  = (t > 0.5f) ? TERR_ROUGH : TERR_FLAT;
    }
}

/* True while a wheel is off the ground: freeze the integrator. */
inline bool terrain_airborne() { return TERR.state == TERR_AIR; }

/* Multiplier for the high-frequency gains (K3/K4). 1.0 = untouched. */
inline float terrain_soften() { return TERR.soften; }

inline void terrain_reset() {
    TERR.drops = 0; TERR.peakG = 0.0f; TERR.rough = 0.0f;
}

#endif // TERRAIN_H
