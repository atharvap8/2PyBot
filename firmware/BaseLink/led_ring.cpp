/*
 * ============================================================
 *  led_ring.cpp — status ring (WS2812, 16 LEDs)
 * ============================================================
 *  Every state has its own colour AND motion so it can be read from across
 *  the room. Priority (highest wins each frame):
 *
 *   test          LT,1 / LT,2 orientation test patterns
 *   FATAL         whole ring flashes red continuously (latched error)
 *   ERROR n       n red LEDs from the front flash (1 per error code)
 *   fall / arm    one-shot events (red flash / green sweep)
 *   self-righting orange dots chasing round
 *   gesture       rainbow
 *   disturbance   amber fast blink on the side it was pushed toward
 *   driving       arc toward travel; green = manual, blue = autopilot
 *   holding       steady dim green ring, bright green front LED
 *   stiff / climb steady amber / violet ring
 *   bubble        red arc front/back while the bubble is limiting (overlay)
 *   lift me       idle, arm requested, lying down: green pulse at the front
 *   ready         idle: white front LED, slow blue breathing
 * ============================================================
 */

#include "led_ring.h"
#include <NeoPixelBus.h>

// Send each complete frame from I2S1 DMA (see original note: RMT is sensitive
// to interrupt latency once Bluetooth and the motor loops are active).
static NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1Ws2812xMethod>
    ring(LED_RING_COUNT, LED_RING_PIN);

static uint32_t lastFrameMs = 0;
static uint8_t  eventActive = 0;
static uint32_t eventT0     = 0;
static int      testMode    = 0;

static float fr[LED_RING_COUNT], fg[LED_RING_COUNT], fb[LED_RING_COUNT];

static inline void clearF() { for (int i = 0; i < LED_RING_COUNT; i++) fr[i] = fg[i] = fb[i] = 0; }
static inline int wrapI(int i) { return ((i % LED_RING_COUNT) + LED_RING_COUNT) % LED_RING_COUNT; }
static inline void addF(int i, float r, float g, float b) { i = wrapI(i); fr[i] += r; fg[i] += g; fb[i] += b; }
static inline void setF(int i, float r, float g, float b) { i = wrapI(i); fr[i] = r; fg[i] = g; fb[i] = b; }
// angle in degrees (0 = robot FRONT, +90 = right side) -> LED index (float)
static inline float angToLed(float deg) {
    const float ofs = deg / (360.0f / LED_RING_COUNT);
    return P.ledFront + (P.ledDir >= 0.0f ? ofs : -ofs);
}
static inline int frontLed() { return wrapI((int)lroundf(angToLed(0))); }
// the k-th LED clockwise (seen from above) from the front
static inline int cwFromFront(int k) { return wrapI(frontLed() + (P.ledDir >= 0.0f ? k : -k)); }
static void arc(float centerDeg, float halfLeds, float r, float g, float b) {
    const float c = angToLed(centerDeg);
    for (int i = 0; i < LED_RING_COUNT; i++) {
        float d = fabsf(i - fmodf(c + LED_RING_COUNT * 4.0f, (float)LED_RING_COUNT));
        d = fminf(d, LED_RING_COUNT - d);
        if (d < halfLeds * 2.0f) {
            const float w = expf(-(d * d) / (halfLeds * halfLeds * 0.7f));
            addF(i, r * w, g * w, b * w);
        }
    }
}
static void fillAll(float r, float g, float b) { for (int i = 0; i < LED_RING_COUNT; i++) addF(i, r, g, b); }
static void showF(float capScale = 1.0f) {
    const float cap = P.ledBright * capScale;
    for (int i = 0; i < LED_RING_COUNT; i++)
        ring.SetPixelColor(i, RgbColor((uint8_t)fminf(fr[i] * cap, cap), (uint8_t)fminf(fg[i] * cap, cap),
                                       (uint8_t)fminf(fb[i] * cap, cap)));
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
    digitalWrite(LED_RING_PIN, LOW);
    pinMode(LED_RING_PIN, OUTPUT);
    delay(1);
    ring.Begin();
    clearF(); showF();
    delay(1);
    for (int t = 0; t < LED_RING_COUNT * 2; t++) {          // boot swirl, ~0.5 s
        clearF();
        for (int i = 0; i < LED_RING_COUNT; i++) {
            float r, g, b;
            hsv((i * 360.0f / LED_RING_COUNT) + t * 20.0f, 1.0f, 0.5f, r, g, b);
            const float fade = 0.15f + 0.85f * ((i + t) % LED_RING_COUNT) / (float)LED_RING_COUNT;
            addF(i, r * fade, g * fade, b * fade);
        }
        showF();
        delay(16);
    }
    clearF(); showF();
}

void leds_event(uint8_t ev) { eventActive = ev; eventT0 = millis(); }
void leds_test(int mode) { testMode = mode; }

void leds_fatalFlash() {
    if (!ring.CanShow()) return;
    clearF();
    if ((millis() / 150) % 2 == 0) fillAll(1.0f, 0.0f, 0.0f);
    showF();
}

// ---- test patterns (physical LED indices, independent of LEDFRONT/LEDDIR) ----
static void drawTest(int mode, uint32_t now) {
    clearF();
    if (mode == 1) {
        // Colour compass: LED 0 white, LED 1 dim yellow (shows counting direction),
        // LED 4 red, LED 8 green, LED 12 blue.
        setF(0, 1.0f, 1.0f, 1.0f);
        setF(1, 0.35f, 0.30f, 0.0f);
        setF(LED_RING_COUNT / 4, 1.0f, 0.0f, 0.0f);
        setF(LED_RING_COUNT / 2, 0.0f, 1.0f, 0.0f);
        setF(3 * LED_RING_COUNT / 4, 0.0f, 0.0f, 1.0f);
    } else if (mode == 2) {
        // Configured orientation: front white (blinking), right side green, back red.
        if ((now / 400) % 2 == 0) arc(0.0f, 0.6f, 1.0f, 1.0f, 1.0f);
        arc(90.0f, 0.6f, 0.0f, 1.0f, 0.0f);
        arc(180.0f, 0.6f, 1.0f, 0.0f, 0.0f);
    }
    showF();
}

void leds_update(const LedInputs& in) {
    const uint32_t now = millis();
    if (now - lastFrameMs < (uint32_t)(1000.0f / (P.ledFps < 1 ? 1 : P.ledFps))) return;
    if (!ring.CanShow()) return;                     // skip a busy frame rather than stall control
    lastFrameMs = now;
    const float t = now / 1000.0f;

    if (testMode) { drawTest(testMode, now); return; }
    clearF();

    // ---------- FATAL: continuous whole-ring red flash ----------
    if (in.fatal) {
        if ((now / 150) % 2 == 0) fillAll(1.0f, 0.0f, 0.0f);
        showF(); return;
    }

    // ---------- one-shot events ----------
    if (eventActive) {
        const float e = (now - eventT0) / 1000.0f;
        if (eventActive == LED_EV_FALL && e < 1.0f) {
            fillAll((e < 0.2f || (e > 0.4f && e < 0.6f)) ? 1.0f : 0.0f, 0.0f, 0.0f);
            showF(); return;
        } else if (eventActive == LED_EV_ARM && e < 0.6f) {
            const float sweep = e / 0.6f * 180.0f;
            arc(sweep, 1.5f, 0.0f, 1.0f, 0.1f); arc(-sweep, 1.5f, 0.0f, 1.0f, 0.1f);
            showF(); return;
        } else if (eventActive == LED_EV_SPEED_HI && e < 0.45f) {
            arc(e / 0.45f * 720.0f, 2.0f, 0.15f, 0.45f, 1.0f);
            showF(); return;
        } else if (eventActive == LED_EV_SPEED_LO && e < 0.45f) {
            const float k = 1.0f - e / 0.45f;
            arc(0.0f, 1.5f + 6.0f * k, 0.05f + 0.10f * k, 0.60f, 0.15f);
            showF(); return;
        } else if ((eventActive == LED_EV_FALL && e >= 1.0f) || (eventActive != LED_EV_FALL && e >= 0.6f)) {
            eventActive = 0;
        }
    }

    // ---------- ERROR n: n red LEDs from the front, flashing ----------
    // Not shown while balancing: a robot that is holding itself up shows that instead.
    if (in.errorCode && !in.balancing && !in.selfRighting) {
        const bool on = (now % 1400) < 700;                       // 0.7 s on / 0.7 s off
        for (int k = 0; k < in.errorCode && k < LED_RING_COUNT; k++)
            if (on) setF(cwFromFront(k), 1.0f, 0.0f, 0.0f);
        if (!on) setF(frontLed(), 0.06f, 0.06f, 0.06f);           // keep the front visible
        showF(); return;
    }

    // ---------- self-uprighting: orange chase ----------
    if (in.selfRighting) {
        const float a = fmodf(t * 540.0f, 360.0f);
        arc(a, 1.0f, 1.0f, 0.35f, 0.0f); arc(a + 180.0f, 1.0f, 1.0f, 0.35f, 0.0f);
        showF(); return;
    }

    // ---------- gesture: rainbow ----------
    if (in.gestActive) {
        for (int i = 0; i < LED_RING_COUNT; i++) {
            float r, g, b; hsv(i * 360.0f / LED_RING_COUNT + t * 240.0f, 1.0f, 0.9f, r, g, b); addF(i, r, g, b);
        }
        showF(); return;
    }

    const bool driving = fabsf(in.fwd) > P.driveDb || fabsf(in.steer) > P.steerDb || in.autopilot;

    if (in.balancing) {
        // ---------- external disturbance: amber fast blink toward the push ----------
        const bool pushed = !driving && (fabsf(in.ex) > 0.05f || fabsf(in.rateF) > 60.0f);
        if (pushed) {
            const bool on = (now % 250) < 125;
            if (on) {
                // ex = pos - hold point: > 0 means pushed forward
                arc(in.ex >= 0 ? 0.0f : 180.0f, 3.0f, 1.0f, 0.45f, 0.0f);
            }
            showF(); return;
        }
        if (driving) {
            // ---------- driving: arc toward travel, green manual / blue autopilot ----------
            const float mag = fminf(sqrtf(in.fwd * in.fwd + in.steer * in.steer), 1.0f);
            // steer > 0 turns left (CCW): arc leans left = negative angle
            const float headDeg = in.fwd < -P.driveDb ? 180.0f - atan2f(-in.steer, -in.fwd) * 57.2958f
                                                     : -atan2f(in.steer, fmaxf(in.fwd, 0.05f)) * 57.2958f;
            const float bri = 0.35f + 0.65f * mag;
            if (in.autopilot) { fillAll(0.0f, 0.0f, 0.06f); arc(headDeg, 1.8f + 2.0f * mag, 0.05f * bri, 0.35f * bri, bri); }
            else              { fillAll(0.0f, 0.05f, 0.0f); arc(headDeg, 1.8f + 2.0f * mag, 0.10f * bri, bri, 0.25f * bri); }
        } else if (in.climbMode) {
            fillAll(0.30f, 0.0f, 0.55f);                           // steady violet
            setF(frontLed(), 0.7f, 0.2f, 1.0f);
        } else if (in.stiffHold) {
            fillAll(0.45f, 0.18f, 0.0f);                           // steady amber
            setF(frontLed(), 1.0f, 0.5f, 0.0f);
        } else {
            // ---------- holding pose: steady green, bright front ----------
            fillAll(0.0f, 0.10f, 0.0f);
            setF(frontLed(), 0.0f, 1.0f, 0.1f);
        }
        // ---------- bubble overlay: red arc where it is limiting ----------
        if (in.bubbleFront) arc(0.0f, 1.6f, 1.0f, 0.0f, 0.0f);
        if (in.bubbleBack)  arc(180.0f, 1.6f, 1.0f, 0.0f, 0.0f);
        showF(); return;
    }

    // ---------- idle, arm requested but lying down: "lift me" ----------
    if (in.armRequested) {
        const float p = 0.15f + 0.85f * (0.5f + 0.5f * sinf(t * 5.0f));
        arc(0.0f, 2.0f, 0.0f, p, 0.15f * p);
        showF(); return;
    }

    // ---------- idle and ready: waiting for arm ----------
    const float breath = 0.04f + 0.08f * (0.5f + 0.5f * sinf(t * 1.6f));
    fillAll(0.0f, breath * 0.3f, breath);
    setF(frontLed(), 0.6f, 0.6f, 0.6f);
    showF();
}
