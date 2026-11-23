/*
 * ============================================================
 *  led_ring.cpp — status ring (WS2812, 16 LEDs)
 * ============================================================
 *  Colour says WHAT, motion says WHERE / HOW FAST.
 *
 *   RED is reserved for errors:
 *     FATAL        whole ring flashes red continuously
 *     ERROR n      n red LEDs from the front flash together (code = count),
 *                  front LED stays dim white so counting has a start
 *     fell         red double flash (then code 8 for 10 s)
 *
 *   Not armed:
 *     no gamepad   error 7 (7 red LEDs)
 *     ready        gamepad connected, waiting for START:
 *                  violet ring, white "heartbeat" double pulse at the front
 *     lift me      START pressed while lying down: green pulse at the front
 *     uprighting   orange dots chasing round
 *
 *   Armed (colour: green = manual, blue = autopilot, violet = climb):
 *     holding      soft breathing ring, bright front "eye" that blinks now and
 *                  then, glow leans toward the side the robot tilts to
 *     forward      two comets run from the left and right sides INTO the front LED
 *                  (while comets run, every other LED is off)
 *     reverse      two comets run from the left and right sides INTO the rear LED
 *     turn right   a comet sweeps the front half from front-left to front-right
 *     turn left    the same sweep right -> left
 *     (forward/reverse and turning overlay; comet speed follows stick/command)
 *     climb mode   one violet LED hops LED by LED toward the travel direction
 *                  (front, back, left or right), faster with speed; others off
 *     pushed       amber fast blink on the side it was pushed toward
 *     bubble       magenta arc front/back while the safety bubble is limiting
 * ============================================================
 */

#include "led_ring.h"
#include <NeoPixelBus.h>

// Send each complete frame from I2S1 DMA (RMT is sensitive to interrupt
// latency once Bluetooth and the motor loops are active).
static NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1Ws2812xMethod>
    ring(LED_RING_COUNT, LED_RING_PIN);

static uint32_t lastFrameMs = 0;
static uint8_t  eventActive = 0;
static uint32_t eventT0     = 0;
static int      testMode    = 0;
static float    phaseTurn = 0, phaseDrive = 0, phaseClimb = 0;   // animation phases (cycles)
static float    lean = 0;                                         // smoothed tilt glow

static float fr[LED_RING_COUNT], fg[LED_RING_COUNT], fb[LED_RING_COUNT];

static inline void clearF() { for (int i = 0; i < LED_RING_COUNT; i++) fr[i] = fg[i] = fb[i] = 0; }
static inline int wrapI(int i) { return ((i % LED_RING_COUNT) + LED_RING_COUNT) % LED_RING_COUNT; }
static inline void addF(int i, float r, float g, float b) { i = wrapI(i); fr[i] += r; fg[i] += g; fb[i] += b; }
static inline void setF(int i, float r, float g, float b) { i = wrapI(i); fr[i] = r; fg[i] = g; fb[i] = b; }
static constexpr float STEP_DEG = 360.0f / LED_RING_COUNT;
// angle (0 = robot FRONT, +90 = right) -> LED index (float)
static inline float angToLed(float deg) {
    const float ofs = deg / STEP_DEG;
    return P.ledFront + (P.ledDir >= 0.0f ? ofs : -ofs);
}
// LED index -> angle in (-180, 180]
static inline float ledToAng(int i) {
    float d = (i - P.ledFront) * STEP_DEG * (P.ledDir >= 0.0f ? 1.0f : -1.0f);
    d = fmodf(d + 540.0f, 360.0f) - 180.0f;
    return d;
}
static inline int frontLed() { return wrapI((int)lroundf(angToLed(0))); }
static inline int cwFromFront(int k) { return wrapI(frontLed() + (P.ledDir >= 0.0f ? k : -k)); }
static void arc(float centerDeg, float halfLeds, float r, float g, float b) {
    const float c = fmodf(angToLed(centerDeg) + LED_RING_COUNT * 8.0f, (float)LED_RING_COUNT);
    for (int i = 0; i < LED_RING_COUNT; i++) {
        float d = fabsf(i - c);
        d = fminf(d, LED_RING_COUNT - d);
        if (d < halfLeds * 2.0f) {
            const float w = expf(-(d * d) / (halfLeds * halfLeds * 0.7f));
            addF(i, r * w, g * w, b * w);
        }
    }
}
// comet: bright head at headDeg with a fading tail trailing opposite to travel (dirDeg = +1/-1)
static void comet(float headDeg, float dir, float r, float g, float b) {
    arc(headDeg, 0.55f, r, g, b);
    arc(headDeg - dir * STEP_DEG, 0.55f, r * 0.35f, g * 0.35f, b * 0.35f);
    arc(headDeg - dir * 2.0f * STEP_DEG, 0.55f, r * 0.12f, g * 0.12f, b * 0.12f);
}
static void fillAll(float r, float g, float b) { for (int i = 0; i < LED_RING_COUNT; i++) addF(i, r, g, b); }
static void showF() {
    const float cap = P.ledBright;
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
}

// Start-up loader: a white comet spinning clockwise, and a dim cyan progress
// bar filling clockwise from the front. Runs from power-on until the Radxa has
// finished its start-up (BR,1) and the controller has calibrated.
static void drawLoader(float progress, float t) {
    clearF();
    if (progress >= 0.0f) {
        const float n = fminf(1.0f, progress) * LED_RING_COUNT;
        for (int k = 0; k < LED_RING_COUNT; k++) {
            const float f = fminf(1.0f, fmaxf(0.0f, n - k));
            if (f > 0) setF(cwFromFront(k), 0.0f, 0.10f * f, 0.16f * f);
        }
    }
    comet(fmodf(t * 300.0f, 360.0f), +1.0f, 0.85f, 0.85f, 0.85f);
    showF();
}
void leds_loading(float progress) {
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < 20 || !ring.CanShow()) return;
    last = now;
    drawLoader(progress, now / 1000.0f);
}

void leds_event(uint8_t ev) { eventActive = ev; eventT0 = millis(); }
void leds_test(int mode) { testMode = mode; }

void leds_fatalFlash() {
    if (!ring.CanShow()) return;
    clearF();
    if ((millis() / 150) % 2 == 0) fillAll(1.0f, 0.0f, 0.0f);
    showF();
}

static void drawTest(int mode, uint32_t now) {
    clearF();
    if (mode == 1) {
        // Colour compass on PHYSICAL indices: 0 white, 1 dim yellow, 4 red, 8 green, 12 blue.
        setF(0, 1.0f, 1.0f, 1.0f);
        setF(1, 0.35f, 0.30f, 0.0f);
        setF(LED_RING_COUNT / 4, 1.0f, 0.0f, 0.0f);
        setF(LED_RING_COUNT / 2, 0.0f, 1.0f, 0.0f);
        setF(3 * LED_RING_COUNT / 4, 0.0f, 0.0f, 1.0f);
    } else if (mode == 2) {
        // Configured orientation: front white (blinking), right green, back red.
        if ((now / 400) % 2 == 0) arc(0.0f, 0.6f, 1.0f, 1.0f, 1.0f);
        arc(90.0f, 0.6f, 0.0f, 1.0f, 0.0f);
        arc(180.0f, 0.6f, 1.0f, 0.0f, 0.0f);
    }
    showF();
}

// Motion animation for an armed robot. fwd/steer in -1..1 (steer > 0 = left).
// (r,g,b) = mode colour. Comet speed grows with the command.
static void motionAnim(float fwd, float steer, float dt, float r, float g, float b) {
    const float af = fminf(fabsf(fwd), 1.0f), as = fminf(fabsf(steer), 1.0f);
    // Only the comets are lit; every other LED stays OFF.
    if (af > 0.02f) {
        phaseDrive = fmodf(phaseDrive + dt * (0.6f + 3.4f * af), 1.0f);
        const float k = af * 0.7f + 0.3f;
        if (fwd > 0) {                                   // forward: both sides run INTO the front LED
            comet(-90.0f + 90.0f * phaseDrive, +1.0f, r * k, g * k, b * k);
            comet( 90.0f - 90.0f * phaseDrive, -1.0f, r * k, g * k, b * k);
        } else {                                         // reverse: both sides run INTO the rear LED
            comet(-90.0f - 90.0f * phaseDrive, -1.0f, r * k, g * k, b * k);
            comet( 90.0f + 90.0f * phaseDrive, +1.0f, r * k, g * k, b * k);
        }
    }
    if (as > 0.02f) {
        phaseTurn = fmodf(phaseTurn + dt * (0.5f + 3.0f * as), 1.0f);
        const float k = as * 0.7f + 0.3f;
        // front half sweep: right turn front-left -> front-right, left turn the reverse
        if (steer < 0) comet(-90.0f + 180.0f * phaseTurn, +1.0f, k, k, k);
        else           comet( 90.0f - 180.0f * phaseTurn, -1.0f, k, k, k);
    }
}

// Climb mode: a single violet LED HOPS one LED at a time (no fade, all others
// off) in the travel direction, faster with speed:
//   forward  two hoppers step from the left/right sides into the front LED
//   reverse  two hoppers step from the sides into the rear LED
//   turning  one hopper steps across the front half (left->right = right turn)
//   stopped  the front LED alone, slow pulse
static inline void litAt(float deg, float v) { setF((int)lroundf(angToLed(deg)), 0.6f * v, 0.05f * v, 1.0f * v); }
static void climbAnim(float fwd, float steer, float dt, float t) {
    const float af = fabsf(fwd), as = fabsf(steer);
    const float speed = fmaxf(af, as);
    if (speed <= 0.02f) { litAt(0.0f, 0.45f + 0.35f * sinf(t * 2.5f)); return; }
    phaseClimb += dt * (2.0f + 10.0f * speed);                 // hops per second
    const int hop = (int)floorf(phaseClimb);
    if (as > af) {
        const int k = hop % 9;                                  // -90 .. +90 in 22.5 deg hops
        litAt(steer < 0 ? -90.0f + k * STEP_DEG : 90.0f - k * STEP_DEG, 1.0f);
    } else {
        const int k = hop % 5;                                  // side -> centre in 4 hops
        if (fwd > 0) { litAt(-90.0f + k * STEP_DEG, 1.0f); litAt(90.0f - k * STEP_DEG, 1.0f); }
        else         { litAt(-90.0f - k * STEP_DEG, 1.0f); litAt(90.0f + k * STEP_DEG, 1.0f); }
    }
}

void leds_update(const LedInputs& in) {
    const uint32_t now = millis();
    if (now - lastFrameMs < (uint32_t)(1000.0f / (P.ledFps < 1 ? 1 : P.ledFps))) return;
    if (!ring.CanShow()) return;                     // skip a busy frame rather than stall control
    const float dt = lastFrameMs ? fminf(0.2f, (now - lastFrameMs) / 1000.0f) : 0.02f;
    lastFrameMs = now;
    const float t = now / 1000.0f;

    if (testMode) { drawTest(testMode, now); return; }
    clearF();

    // ---------- FATAL ----------
    if (in.fatal) {
        if ((now / 150) % 2 == 0) fillAll(1.0f, 0.0f, 0.0f);
        showF(); return;
    }

    // ---------- one-shot events ----------
    if (eventActive) {
        const float e = (now - eventT0) / 1000.0f;
        if (eventActive == LED_EV_FALL && e < 0.35f) {
            fillAll(1.0f - e / 0.35f, 0.0f, 0.0f);              // one short red flash, then the "fell" pattern
            showF(); return;
        } else if (eventActive == LED_EV_READY && e < 1.2f) {
            // ready: green fills both ways from the front, holds, fades
            const float fill = fminf(1.0f, e / 0.5f) * 180.0f, fade = e < 0.7f ? 1.0f : 1.0f - (e - 0.7f) / 0.5f;
            for (int i = 0; i < LED_RING_COUNT; i++) if (fabsf(ledToAng(i)) <= fill + 0.1f) setF(i, 0.0f, 0.8f * fade, 0.2f * fade);
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
        } else if ((eventActive == LED_EV_FALL && e >= 0.35f) || (eventActive == LED_EV_READY && e >= 1.2f) ||
                   (eventActive != LED_EV_FALL && eventActive != LED_EV_READY && e >= 0.6f)) {
            eventActive = 0;
        }
    }

    // ---------- start-up / Radxa not ready: loader ----------
    if (in.loading && !in.balancing && !in.selfRighting && !(in.errorCode && in.errorCode != LED_ERR_PAD)) {
        drawLoader(-1.0f, t); return;
    }

    // ---------- fell: red glow on the side that hit the ground, breathing ----------
    if (in.errorCode == LED_ERR_FELL && !in.balancing && !in.selfRighting) {
        const float b = 0.25f + 0.75f * (0.5f + 0.5f * sinf(t * 3.0f));
        arc(in.pitchF >= 0.0f ? 0.0f : 180.0f, 2.6f, b, 0.0f, 0.0f);
        showF(); return;
    }

    // ---------- ERROR n: n red LEDs, counted clockwise from the front, flash together ----------
    if (in.errorCode && !in.balancing && !in.selfRighting) {
        if ((now % 1200) < 600)
            for (int k = 0; k < in.errorCode && k < LED_RING_COUNT; k++) setF(cwFromFront(k), 1.0f, 0.0f, 0.0f);
        showF(); return;
    }

    // ---------- self-uprighting: orange chase ----------
    if (in.selfRighting) {
        const float a = fmodf(t * 540.0f, 360.0f);
        comet(a, +1.0f, 1.0f, 0.35f, 0.0f); comet(a + 180.0f, +1.0f, 1.0f, 0.35f, 0.0f);
        showF(); return;
    }

    // ---------- gesture: rainbow ----------
    if (in.gestActive) {
        for (int i = 0; i < LED_RING_COUNT; i++) {
            float r, g, b; hsv(i * 360.0f / LED_RING_COUNT + t * 240.0f, 1.0f, 0.9f, r, g, b); addF(i, r, g, b);
        }
        showF(); return;
    }

    if (in.balancing) {
        const float fwd = fabsf(in.fwd) > P.driveDb ? in.fwd : 0.0f;
        const float steer = fabsf(in.steer) > P.steerDb ? in.steer : 0.0f;
        const bool moving = fwd != 0.0f || steer != 0.0f;
        // ---------- pushed: amber fast blink toward the push ----------
        if (!moving && !in.autopilot && (fabsf(in.ex) > 0.05f || fabsf(in.rateF) > 60.0f)) {
            if ((now % 250) < 125) arc(in.ex >= 0 ? 0.0f : 180.0f, 3.0f, 1.0f, 0.45f, 0.0f);
            showF(); return;
        }
        if (in.climbMode) {
            climbAnim(fwd, steer, dt, t);
        } else if (moving) {
            if (in.autopilot) motionAnim(fwd, steer, dt, 0.10f, 0.45f, 1.0f);   // blue
            else              motionAnim(fwd, steer, dt, 0.05f, 1.0f, 0.35f);   // green
        } else {
            // ---------- holding: lively breathing, blinking front eye, tilt glow ----------
            const bool ap = in.autopilot;
            const float r0 = ap ? 0.05f : 0.0f, g0 = ap ? 0.25f : 1.0f, b0 = ap ? 1.0f : 0.25f;
            const float breath = 0.06f + 0.06f * (0.5f + 0.5f * sinf(t * 2.2f));
            fillAll(r0 * breath, g0 * breath, b0 * breath);
            lean += 0.15f * (fmaxf(-1.0f, fminf(1.0f, in.pitchF / 6.0f)) - lean);
            if (fabsf(lean) > 0.08f) arc(lean > 0 ? 0.0f : 180.0f, 2.0f, r0 * 0.35f * fabsf(lean), g0 * 0.35f * fabsf(lean), b0 * 0.35f * fabsf(lean));
            const float blinkT = fmodf(t, 4.3f);
            const bool blink = blinkT < 0.10f || (blinkT > 0.22f && blinkT < 0.30f);
            if (!blink) setF(frontLed(), r0, g0, b0);
            if (in.stiffHold) { arc(90.0f, 0.7f, 0.6f, 0.3f, 0.0f); arc(-90.0f, 0.7f, 0.6f, 0.3f, 0.0f); }
        }
        // ---------- bubble: magenta arc where it is limiting ----------
        if (in.bubbleFront) arc(0.0f, 1.4f, 0.9f, 0.0f, 0.7f);
        if (in.bubbleBack)  arc(180.0f, 1.4f, 0.9f, 0.0f, 0.7f);
        showF(); return;
    }

    // ---------- START pressed while lying down: "lift me" ----------
    if (in.armRequested) {
        const float p = 0.15f + 0.85f * (0.5f + 0.5f * sinf(t * 5.0f));
        arc(0.0f, 2.0f, 0.0f, p, 0.15f * p);
        showF(); return;
    }

    // ---------- connected, waiting for START: violet ring + white heartbeat ----------
    const float hb = fmodf(t, 1.6f);
    const float beat = (hb < 0.12f) ? 1.0f : (hb > 0.25f && hb < 0.37f) ? 0.6f : 0.0f;
    fillAll(0.10f, 0.0f, 0.18f);
    arc(0.0f, 0.9f, beat, beat, beat);
    showF();
}
