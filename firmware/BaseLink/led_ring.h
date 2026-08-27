/*
 * ============================================================
 *  led_ring.h — WS2812 16-LED expression ring (GPIO 15)
 * ============================================================
 *  Rendering is done with NeoPixelBus over the ESP32 RMT
 *  peripheral: the WS2812 timing is generated in hardware, no
 *  interrupts are ever disabled, so the 20 kHz stepper ISR and
 *  the 200 Hz balance loop are completely unaffected.
 *
 *  The main loop calls leds_update(li) every iteration; frames
 *  are rate-limited to LED_FPS internally and each frame costs
 *  microseconds of CPU. Library: "NeoPixelBus by Makuna" >= 2.8.
 * ============================================================
 */

#ifndef LED_RING_H
#define LED_RING_H

#include <Arduino.h>
#include "config.h"

// One-shot events (overlay animations)
#define LED_EV_ARM       1   // green wake-up sweep
#define LED_EV_FALL      2   // red flash + fade ("ouch")
#define LED_EV_SPEED_HI  3   // RB: fast blue double-lap ("sport")
#define LED_EV_SPEED_LO  4   // LB: green arc settling at the front ("easy")

struct LedInputs {
    bool  balancing;     // state == STATE_BALANCING
    bool  armRequested;  // motorsRequested (waiting to be stood up)
    float fwd;           // arbitrated cmdFwd  -1..1
    float steer;         // arbitrated cmdSteer -1..1
    float velF;          // measured forward velocity m/s
    float pitchF;        // deg
    float ex;            // posM - xRef (m) -> "I got pushed" detector
    bool  stiffHold;
    bool  climbMode;     // cliff/climb mode active -> violet grip display
    bool  gestActive;    // any gesture playing -> party mode
    bool  radxaFresh;    // Cubie currently has authority (vision override)
    bool  padConnected;
};

void leds_begin();                       // boot swirl
void leds_update(const LedInputs& in);   // call every loop, self rate-limited
void leds_event(uint8_t ev);             // LED_EV_ARM / LED_EV_FALL

#endif // LED_RING_H
