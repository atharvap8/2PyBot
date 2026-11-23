/*
 * ============================================================
 *  led_ring.h — WS2812 16-LED expression ring (GPIO 15)
 * ============================================================
 *  NeoPixelBus sends complete WS2812 frames through ESP32 I2S1 DMA,
 *  avoiding interrupt-refilled RMT transmission during balancing.
 *  Rendering is rate-limited; a busy frame is skipped rather than
 *  waiting in the control loop. GPIO32 uses separate LEDC hardware.
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
#include "params.h"

// One-shot events (overlay animations)
#define LED_EV_ARM       1   // green wake-up sweep
#define LED_EV_FALL      2   // red flash + fade ("ouch")
#define LED_EV_SPEED_HI  3   // RB: fast blue double-lap ("sport")
#define LED_EV_SPEED_LO  4   // LB: green arc settling at the front ("easy")
#define LED_EV_READY     5   // start-up complete: green fill from the front, then fade

// Error codes: the NUMBER of red LEDs that flash (counted clockwise from the front).
#define LED_ERR_NONE        0
#define LED_ERR_MOTOR_LINK  1   // a hub motor does not answer on RS485
#define LED_ERR_MOTOR_FAULT 2   // a hub motor reports a confirmed fault
#define LED_ERR_MOTOR_MODE  3   // a hub motor is not in current mode / not ready
#define LED_ERR_IMU         4   // IMU samples stale
#define LED_ERR_HOST        5   // waiting for the Radxa (boot gate)
#define LED_ERR_MAINT       6   // firmware maintenance lock active
#define LED_ERR_PAD         7   // gamepad not connected
#define LED_ERR_FELL        8   // last disarm: fell / rate spike (shown 10 s)
#define LED_ERR_SELFRIGHT   9   // last disarm: self-uprighting aborted (shown 10 s)

struct LedInputs {
    bool  balancing;     // state == STATE_BALANCING
    bool  armRequested;  // motorsRequested (waiting to be stood up)
    float fwd;           // arbitrated cmdFwd  -1..1
    float steer;         // arbitrated cmdSteer -1..1
    float velF;          // measured forward velocity m/s
    float pitchF;        // deg
    float rateF;         // deg/s
    float ex;            // posM - xRef (m) -> "I got pushed" detector
    bool  stiffHold;
    bool  climbMode;     // cliff/climb mode active -> violet grip display
    bool  gestActive;    // any gesture playing -> party mode
    bool  radxaFresh;    // Cubie currently has authority (vision override)
    bool  padConnected;
    bool  selfRighting;  // state == STATE_SELF_RIGHTING
    bool  autopilot;     // navigation lease granted
    bool  bubbleFront;   // bubble is limiting forward motion
    bool  bubbleBack;    // bubble is limiting reverse motion
    uint8_t errorCode;   // LED_ERR_*
    bool  fatal;         // latched error: whole ring flashes red continuously
    bool  loading;       // Radxa not ready (BR,0): start-up loader
};

void leds_begin();                       // ring off (no swirl)
void leds_loading(float progress);       // one loader frame (blocking start-up code calls this); <0 = no progress bar
void leds_update(const LedInputs& in);   // call every loop, self rate-limited
void leds_event(uint8_t ev);             // LED_EV_ARM / LED_EV_FALL
void leds_test(int mode);                // 0 off, 1 colour compass, 2 configured front
void leds_fatalFlash();                  // blocking red flash frame for halt loops

#endif // LED_RING_H
