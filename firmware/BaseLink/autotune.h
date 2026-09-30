/*
 * ============================================================
 *  autotune.h — self-tuning routines that run ON the robot
 * ============================================================
 *  Triggered from the app / GUI / serial. Each is a small state
 *  machine stepped once per control loop, so nothing blocks the
 *  200 Hz balance loop and every routine can be aborted instantly.
 *
 *  COMMANDS      (all reply with A,<line> progress messages)
 *    AT,wobble   damp an oscillation: measures pitch RMS, walks K3/K4
 *                down until it settles, then stops. Robot must be
 *                BALANCING and stationary.
 *    AT,trim     find the true balance point: averages the pitch the
 *                robot actually holds and folds it into TRIM.
 *    AT,radius   loaded wheel radius. Say AT,radius then push the robot
 *                exactly 2.00 m by hand, then AT,done.
 *    AT,stall    find the real speed ceiling. WHEELS OFF THE GROUND:
 *                ramps the motors until the encoders stop following,
 *                then sets MAXSPD to 80 % of that.
 *    AT,stop     abort whatever is running, restore anything staged
 *    AT,?        status
 *
 *  SAFETY
 *   - every routine has a hard timeout and an iteration cap
 *   - gains are never moved outside the params.h bounds, and never
 *     below AT_GAIN_FLOOR of where they started
 *   - a fall (state leaving BALANCING) aborts and restores originals
 *   - nothing is written to NVS: you review the result and press Save
 * ============================================================
 */

#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <Arduino.h>
#include "params.h"

#define AT_GAIN_FLOOR      0.40f   // never below 40 % of the starting gain
#define AT_WOBBLE_WINDOW   1500    // ms per measurement window
#define AT_WOBBLE_STEPS    8       // max reduction iterations
#define AT_WOBBLE_TARGET   0.45f   // deg RMS considered "settled"
#define AT_TRIM_WINDOW     4000    // ms of averaging
#define AT_TRIM_PASSES     3
#define AT_STALL_STEP      250.0f  // usteps/s per ramp tick
#define AT_STALL_TICK      120     // ms between ramp ticks
#define AT_TIMEOUT_MS      45000UL

enum AtMode : uint8_t { AT_IDLE, AT_WOBBLE, AT_TRIM, AT_RADIUS, AT_STALL };

struct AutoTune {
    AtMode   mode = AT_IDLE;
    uint8_t  step = 0;
    uint32_t t0 = 0, tWin = 0;
    // measurement accumulators
    double   sum = 0, sumSq = 0;
    uint32_t n = 0;
    float    best = 0;
    // saved originals so an abort always restores
    float    k3_0 = 0, k4_0 = 0, trim_0 = 0, spd_0 = 0;
    int64_t  encStart = 0;
    float    ramp = 0;
    bool     staged = false;      // a result is waiting for the user to keep it
};

static AutoTune AT;

// The .ino provides these (it owns the state machine and the hardware).
extern float pitchF, rateF, velF, posM, cmdFwd, cmdSteer;
bool at_isBalancing();
void at_setWheelSpeed(float stepsPerSec);     // raw, bypasses the controller
void at_motorsOff();

inline void at_say(const char* fmt, ...) {
    char buf[160];
    va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    Serial.printf("A,%s\n", buf);
}

inline void at_restore(const char* why) {
    if (AT.mode == AT_WOBBLE) { P.k3 = AT.k3_0; P.k4 = AT.k4_0; }
    if (AT.mode == AT_TRIM)   { P.pitchTrim = AT.trim_0; }
    if (AT.mode == AT_STALL)  { at_setWheelSpeed(0); at_motorsOff(); P.maxSpdSt = AT.spd_0; }
    at_say("abort,%s", why);
    AT.mode = AT_IDLE;
}

inline void at_finish(const char* summary) {
    if (AT.mode == AT_STALL) { at_setWheelSpeed(0); at_motorsOff(); }
    at_say("done,%s", summary);
    at_say("note,nothing saved yet -- press Save to keep it, or Reload to discard");
    AT.mode = AT_IDLE;
    AT.staged = true;
}

inline void at_begin(AtMode m) {
    AT = AutoTune();
    AT.mode = m; AT.t0 = millis(); AT.tWin = millis();
    AT.k3_0 = P.k3; AT.k4_0 = P.k4; AT.trim_0 = P.pitchTrim; AT.spd_0 = P.maxSpdSt;
}

// ------------------------------------------------------------
//  called once per control loop from the .ino
// ------------------------------------------------------------
inline void at_update(float dt) {
    if (AT.mode == AT_IDLE) return;
    uint32_t now = millis();

    if (now - AT.t0 > AT_TIMEOUT_MS) { at_restore("timeout"); return; }
    if (AT.mode != AT_RADIUS && AT.mode != AT_STALL && !at_isBalancing()) {
        at_restore("robot stopped balancing"); return;
    }
    if (AT.mode != AT_RADIUS &&
        (fabsf(cmdFwd) > 0.05f || fabsf(cmdSteer) > 0.05f)) {
        at_restore("stick moved -- tune with hands off"); return;
    }

    switch (AT.mode) {

    // ---------- WOBBLE: walk K3/K4 down until the pitch RMS settles ----------
    case AT_WOBBLE: {
        AT.sumSq += (double)pitchF * pitchF;
        AT.n++;
        if (now - AT.tWin < AT_WOBBLE_WINDOW) return;
        float rms = sqrtf((float)(AT.sumSq / (AT.n ? AT.n : 1)));
        at_say("wobble,%u,rms,%.3f,K3,%.2f,K4,%.2f", AT.step, rms, P.k3, P.k4);
        AT.sumSq = 0; AT.n = 0; AT.tWin = now;

        if (rms <= AT_WOBBLE_TARGET) {
            char s[96];
            snprintf(s, sizeof(s), "settled at rms %.3f deg, K3 %.2f K4 %.2f", rms, P.k3, P.k4);
            at_finish(s); return;
        }
        if (++AT.step > AT_WOBBLE_STEPS) { at_finish("reduction limit reached"); return; }

        // Pitch oscillation on a compliant tyre is a high-frequency problem:
        // back off the rate term harder than the angle term.
        float k3n = P.k3 * 0.93f, k4n = P.k4 * 0.88f;
        if (fabsf(k3n) < fabsf(AT.k3_0) * AT_GAIN_FLOOR ||
            fabsf(k4n) < fabsf(AT.k4_0) * AT_GAIN_FLOOR) {
            at_finish("hit the 40% gain floor -- the wobble is mechanical, not gains");
            return;
        }
        params_set("K3", k3n, true);
        params_set("K4", k4n, true);
        break;
    }

    // ---------- TRIM: the pitch it actually holds IS the offset ----------
    case AT_TRIM: {
        AT.sum += pitchF; AT.n++;
        if (now - AT.tWin < AT_TRIM_WINDOW) return;
        float mean = (float)(AT.sum / (AT.n ? AT.n : 1));
        AT.sum = 0; AT.n = 0; AT.tWin = now;
        // pitchF = sign * (raw - trim); holding a non-zero pitchF means the
        // trim is off by exactly that much, in the same sign convention.
        float newTrim = P.pitchTrim + P.pitchSign * mean;
        at_say("trim,%u,mean,%.3f,trim,%.3f", AT.step, mean, newTrim);
        if (params_set("TRIM", newTrim, true) != 0) {
            at_finish("trim would leave its safe range"); return;
        }
        if (++AT.step >= AT_TRIM_PASSES || fabsf(mean) < 0.05f) {
            char s[80];
            snprintf(s, sizeof(s), "trim %.3f deg (residual %.3f)", P.pitchTrim, mean);
            at_finish(s);
        }
        break;
    }

    // ---------- RADIUS: encoder counts over a hand-pushed 2.00 m ----------
    case AT_RADIUS:
        if (now - AT.tWin > 1000) {
            AT.tWin = now;
            float travelled = (posM - (float)AT.encStart);
            at_say("radius,push 2.00 m by hand, then AT,done  (moved %.2f m so far)", travelled);
        }
        break;

    // ---------- STALL: ramp until the encoders stop following ----------
    case AT_STALL: {
        if (now - AT.tWin < AT_STALL_TICK) return;
        AT.tWin = now;
        AT.ramp += AT_STALL_STEP;
        at_setWheelSpeed(AT.ramp);
        float expected = AT.ramp / P.stepsPerM;          // m/s the wheels should do
        float actual   = fabsf(velF);
        at_say("stall,cmd,%.0f,expect,%.2f,actual,%.2f", AT.ramp, expected, actual);
        if (AT.ramp > 400 && actual < expected * 0.70f) {
            float safe = AT.ramp * 0.80f;
            params_set("MAXSPD", safe, true);
            char s[96];
            snprintf(s, sizeof(s), "stalled at %.0f usteps/s -> MAXSPD set to %.0f (80%%)",
                     AT.ramp, safe);
            at_finish(s); return;
        }
        if (AT.ramp >= P.maxSpdSt * 1.6f || AT.ramp > 14000) {
            at_finish("no stall found within the search range");
            return;
        }
        break;
    }
    default: break;
    }
}

// ------------------------------------------------------------
//  command entry point. Returns true if the line was an AT command.
// ------------------------------------------------------------
inline bool at_handleLine(char* line) {
    if (strncasecmp(line, "AT,", 3) != 0) return false;
    const char* a = line + 3;

    if (!strcasecmp(a, "stop"))  { if (AT.mode != AT_IDLE) at_restore("user"); else at_say("idle"); return true; }
    if (!strcasecmp(a, "?"))     {
        const char* n[] = {"idle","wobble","trim","radius","stall"};
        at_say("status,%s,step,%u", n[AT.mode], AT.step); return true;
    }
    if (AT.mode != AT_IDLE) { at_say("busy,%d", (int)AT.mode); return true; }

    if (!strcasecmp(a, "wobble")) {
        if (!at_isBalancing()) { at_say("refused,must be balancing"); return true; }
        at_begin(AT_WOBBLE);
        at_say("start,wobble,K3,%.2f,K4,%.2f,target rms %.2f deg", P.k3, P.k4, AT_WOBBLE_TARGET);
        return true;
    }
    if (!strcasecmp(a, "trim")) {
        if (!at_isBalancing()) { at_say("refused,must be balancing"); return true; }
        at_begin(AT_TRIM);
        at_say("start,trim,hold still, hands off, %d passes", AT_TRIM_PASSES);
        return true;
    }
    if (!strcasecmp(a, "radius")) {
        at_begin(AT_RADIUS);
        AT.encStart = (int64_t)posM;
        AT.encStart = 0;                  // posM is metres; capture the origin
        AT.best = posM;
        at_say("start,radius,now push the robot EXACTLY 2.00 m, then send AT,done");
        return true;
    }
    if (!strcasecmp(a, "done")) {
        if (AT.mode != AT_RADIUS) { at_say("refused,no radius run active"); return true; }
        float measured = fabsf(posM - AT.best);          // metres per the OLD radius
        if (measured < 0.20f) { at_restore("moved less than 20 cm"); return true; }
        // reported distance scales linearly with the assumed radius
        float newR = P.wheelR * (2.00f / measured);
        char s[110];
        if (params_set("WHEELR", newR, true) != 0) {
            at_restore("computed radius is outside the safe range"); return true;
        }
        snprintf(s, sizeof(s), "reported %.3f m for a real 2.00 m -> WHEELR %.4f m (was %.4f)",
                 measured, newR, AT.spd_0 > 0 ? P.wheelR : P.wheelR);
        at_finish(s);
        return true;
    }
    if (!strcasecmp(a, "stall")) {
        if (at_isBalancing()) { at_say("refused,disarm first -- WHEELS OFF THE GROUND"); return true; }
        at_begin(AT_STALL);
        AT.ramp = 0;
        at_say("start,stall,WHEELS MUST BE OFF THE GROUND -- ramping now");
        return true;
    }
    at_say("unknown,%s", a);
    return true;
}

#endif // AUTOTUNE_H
