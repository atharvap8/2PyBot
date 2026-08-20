/*
 * ============================================================
 *  led_ring.cpp — personality engine for the WS2812 ring
 * ============================================================
 *  Priority (highest wins each frame):
 *    1. event overlay      fall = red ouch, arm = green wake sweep
 *    2. party              any gesture -> spinning rainbow
 *    3. vision brake       Cubie override while still rolling ->
 *                          urgent red double-pulse (hazard lights)
 *    4. shy                pushed off its spot -> pink blush on the
 *                          "cheeks" that fades as it walks back
 *    5. driving            arc of light pointing where it's going;
 *                          brighter + wider with speed, orange when
 *                          reversing, arc leans into the turn
 *    6. stiff hold         determined amber breathing (fast)
 *    7. balancing idle     calm cyan breathing + a random twinkle,
 *                          warm tint grows with tilt "effort"
 *    8. waiting for pickup armRequested but lying down -> green
 *                          "help me up" pulse at the front
 *    9. sleeping           slow drifting warm ember + occasional
 *                          sleepy double-blink
 * ============================================================
 */

#include "led_ring.h"
#include <NeoPixelBus.h>

// RMT channel 0; timing generated in hardware, zero IRQ impact.
static NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod>
    ring(LED_RING_COUNT, LED_RING_PIN);

static uint32_t lastFrameMs = 0;
static uint8_t  eventActive = 0;
static uint32_t eventT0     = 0;
static bool     padWasOn    = false;
static uint32_t padBlinkT0  = 0;

// float pixel buffer so effects can blend before quantizing
static float fr[LED_RING_COUNT], fg[LED_RING_COUNT], fb[LED_RING_COUNT];

static inline void clearF() {
    for (int i = 0; i < LED_RING_COUNT; i++) fr[i] = fg[i] = fb[i] = 0;
}
static inline void addF(int i, float r, float g, float b) {
    i = ((i % LED_RING_COUNT) + LED_RING_COUNT) % LED_RING_COUNT;
    fr[i] += r; fg[i] += g; fb[i] += b;
}
// angle in degrees (0 = robot FRONT, +90 = right side) -> LED index (float)
static inline float angToLed(float deg) {
    float per = 360.0f / LED_RING_COUNT;
    float ofs = deg / per;
    return LED_FRONT_INDEX + (LED_DIR_CW ? ofs : -ofs);
}
// gaussian-ish arc centered on 'centerDeg', half-width in LEDs
static void arc(float centerDeg, float halfLeds, float r, float g, float b) {
    float c = angToLed(centerDeg);
    for (int i = 0; i < LED_RING_COUNT; i++) {
        float d = fabsf(i - c);
        d = fminf(d, LED_RING_COUNT - d);           // wraparound distance
        if (d < halfLeds * 2.0f) {
            float w = expf(-(d * d) / (halfLeds * halfLeds * 0.7f));
            addF(i, r * w, g * w, b * w);
        }
    }
}
static void fillAll(float r, float g, float b) {
    for (int i = 0; i < LED_RING_COUNT; i++) addF(i, r, g, b);
}
static void showF() {
    float cap = LED_MAX_BRIGHT;
    for (int i = 0; i < LED_RING_COUNT; i++) {
        ring.SetPixelColor(i, RgbColor(
            (uint8_t)fminf(fr[i] * cap, cap),
            (uint8_t)fminf(fg[i] * cap, cap),
            (uint8_t)fminf(fb[i] * cap, cap)));
    }
    ring.Show();
}
static void hsv(float h, float s, float v, float& r, float& g, float& b) {
    h = fmodf(h, 360.0f) / 60.0f;
    float c = v * s, x = c * (1 - fabsf(fmodf(h, 2.0f) - 1)), m = v - c;
    float R = 0, G = 0, B = 0;
    if      (h < 1) { R = c; G = x; }
    else if (h < 2) { R = x; G = c; }
    else if (h < 3) { G = c; B = x; }
    else if (h < 4) { G = x; B = c; }
    else if (h < 5) { R = x; B = c; }
    else            { R = c; B = x; }
    r = R + m; g = G + m; b = B + m;
}

void leds_begin() {
    ring.Begin();
    // boot swirl: one rainbow lap, ~0.5 s, blocking is fine in setup()
    for (int t = 0; t < LED_RING_COUNT * 2; t++) {
        clearF();
        for (int i = 0; i < LED_RING_COUNT; i++) {
            float r, g, b;
            hsv((i * 360.0f / LED_RING_COUNT) + t * 20.0f, 1.0f, 0.5f, r, g, b);
            float fade = 0.15f + 0.85f * ((i + t) % LED_RING_COUNT) / (float)LED_RING_COUNT;
            addF(i, r * fade, g * fade, b * fade);
        }
        showF();
        delay(16);
    }
    clearF(); showF();
}

void leds_event(uint8_t ev) { eventActive = ev; eventT0 = millis(); }

void leds_update(const LedInputs& in) {
    uint32_t now = millis();
    if (now - lastFrameMs < (1000 / LED_FPS)) return;
    lastFrameMs = now;
    float t = now / 1000.0f;
    clearF();

    // pad connect greeting: two quick green winks (overlay, non-exclusive)
    if (in.padConnected && !padWasOn) padBlinkT0 = now;
    padWasOn = in.padConnected;
    bool greeting = (now - padBlinkT0) < 600 && padBlinkT0 != 0;

    // ---------- 1. one-shot events ----------
    if (eventActive) {
        float e = (now - eventT0) / 1000.0f;
        if (eventActive == LED_EV_FALL) {
            if (e < 1.4f) {
                float flash = (e < 0.25f) ? 1.0f : expf(-(e - 0.25f) * 3.0f);
                float wobble = 0.75f + 0.25f * sinf(t * 40.0f);
                fillAll(flash * wobble, 0.02f * flash, 0.0f);
                showF(); return;
            }
            eventActive = 0;
        } else if (eventActive == LED_EV_ARM) {
            if (e < 0.6f) {
                float sweep = e / 0.6f * 360.0f;
                arc(sweep, 2.5f, 0.1f, 1.0f, 0.25f);
                arc(-sweep, 2.5f, 0.1f, 1.0f, 0.25f);
                showF(); return;
            }
            eventActive = 0;
        } else eventActive = 0;
    }

    // ---------- 2. party (any gesture) ----------
    if (in.gestActive) {
        for (int i = 0; i < LED_RING_COUNT; i++) {
            float r, g, b;
            hsv(i * 360.0f / LED_RING_COUNT + t * 240.0f, 1.0f, 0.9f, r, g, b);
            addF(i, r, g, b);
        }
        showF(); return;
    }

    bool driving = fabsf(in.fwd) > DRIVE_DEADBAND || fabsf(in.steer) > STEER_DEADBAND;

    // ---------- 3. vision brake (Cubie override while rolling) ----------
    if (in.radxaFresh && in.balancing && !driving && fabsf(in.velF) > 0.10f) {
        float pulse = (sinf(t * 18.0f) > 0.2f) ? 1.0f : 0.15f;
        fillAll(pulse, pulse * 0.12f, 0.0f);
        showF(); return;
    }

    // ---------- 4. shy (pushed off the hold point) ----------
    if (in.balancing && !driving && fabsf(in.ex) > 0.045f) {
        float howFar = fminf(fabsf(in.ex) / EX_CLAMP_M, 1.0f);
        float blush = 0.35f + 0.65f * (0.5f + 0.5f * sinf(t * 9.0f));
        float wiggle = sinf(t * 5.0f) * 25.0f;                 // averted "eyes"
        arc( 90.0f + wiggle, 2.2f, blush * howFar, blush * 0.18f * howFar, blush * 0.28f * howFar);
        arc(-90.0f - wiggle, 2.2f, blush * howFar, blush * 0.18f * howFar, blush * 0.28f * howFar);
        // faint white "looking away" dot opposite the displacement
        arc(in.ex > 0 ? 180.0f : 0.0f, 1.0f, 0.12f, 0.12f, 0.12f);
        showF(); return;
    }

    // ---------- 5. driving: heading arc ----------
    if (in.balancing && driving) {
        float mag = fminf(sqrtf(in.fwd * in.fwd + in.steer * in.steer), 1.0f);
        float headDeg = atan2f(in.steer, in.fwd) * 180.0f / PI;  // 0 front, +90 right
        float width = 1.6f + 3.0f * mag;
        bool reversing = in.fwd < -DRIVE_DEADBAND;
        float bri = 0.25f + 0.75f * mag;
        if (reversing) arc(headDeg, width, bri, bri * 0.25f, 0.0f);        // amber tail
        else           arc(headDeg, width, bri * 0.15f, bri * 0.75f, bri); // cyan-white beam
        // speed sparkles trailing the arc
        if (fabsf(in.velF) > 0.25f && (now / 90) % 3 == 0)
            arc(headDeg + 180.0f, 0.8f, 0.10f, 0.10f, 0.12f);
        if (greeting) fillAll(0.0f, 0.15f, 0.03f);
        showF(); return;
    }

    // ---------- 6. stiff hold: determined amber ----------
    if (in.balancing && in.stiffHold) {
        float b = 0.45f + 0.35f * sinf(t * 6.0f);
        fillAll(b, b * 0.45f, 0.0f);
        showF(); return;
    }

    // ---------- 7. balancing idle: calm breathing + twinkle ----------
    if (in.balancing) {
        float breath = 0.10f + 0.14f * (0.5f + 0.5f * sinf(t * 1.6f));
        float effort = fminf(fabsf(in.pitchF) / 8.0f, 1.0f);   // warm tint with tilt
        fillAll(breath * (0.15f + 0.85f * effort),
                breath * (0.55f + 0.15f * (1 - effort)),
                breath * (0.85f * (1 - effort)));
        if ((now / 130) % 23 == (uint32_t)(LED_RING_COUNT * 1.3f) % 23)   // rare twinkle
            addF(now / 130 % LED_RING_COUNT, 0.5f, 0.5f, 0.6f);
        if (greeting) { arc(0, 2.0f, 0.0f, 0.8f, 0.15f); }
        showF(); return;
    }

    // ---------- 8. waiting to be stood up ----------
    if (in.armRequested) {
        float p = 0.15f + 0.5f * (0.5f + 0.5f * sinf(t * 3.5f));
        arc(0.0f, 2.5f, 0.05f * p, p, 0.2f * p);   // green "lift me!" at the front
        showF(); return;
    }

    // ---------- 9. sleeping ----------
    float drift = fmodf(t * 18.0f, 360.0f);        // slow wandering ember
    arc(drift, 1.4f, 0.10f, 0.045f, 0.0f);
    float zz = fmodf(t, 6.0f);                      // sleepy double-blink
    if (zz < 0.12f || (zz > 0.25f && zz < 0.37f)) fillAll(0.05f, 0.03f, 0.0f);
    if (greeting) fillAll(0.0f, 0.10f, 0.02f);
    showF();
}
