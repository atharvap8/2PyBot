/*
 * ============================================================
 *  payload.h — camera payload: 2x MG90S servo + flashlight LED
 * ============================================================
 *  Yaw servo   GPIO 0   right stick X   (rate controlled)
 *  Zoom servo  GPIO 12  right stick Y   (rate controlled)
 *  Torch LED   GPIO 2   RB toggles, D-pad LEFT/RIGHT dims
 *
 *  Everything here runs on the LEDC peripheral, driven straight
 *  from the registers. No servo library, so nothing can quietly
 *  grab a timer behind our back and there is no version drift
 *  between the 2.x core (Bluepad32) and the 3.x core.
 *
 *  LEDC allocation, chosen so the two groups never share a timer:
 *    ch 4 + ch 5 -> timer 2, 50 Hz, 16-bit   (both servos)
 *    ch 6        -> timer 3, 20 kHz, 10-bit  (torch)
 *  20 kHz is above the audible range and far above any camera
 *  shutter, so the light does not band on video.
 *
 *  None of this touches the 20 kHz stepper ISR: LEDC is separate
 *  silicon from the general purpose timer that drives the steppers,
 *  and every call below is a register write with no busy waiting.
 *
 *  THE TORCH DUTY IS HARD CAPPED. Every path to the hardware goes
 *  through _torchApply(), which clamps to TORCH_MAX_PCT, and the
 *  #error below refuses to build if that constant is ever raised
 *  past 50. Serial and pad input cannot get around either one.
 * ============================================================
 */

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <Arduino.h>
#include "driver/gpio.h"
#include "config.h"
#include "params.h"

#if TORCH_MAX_PCT > 90
#error "TORCH_MAX_PCT above 90 WILL COOK THE LED"
#endif

// ------------------------------------------------------------
//  LEDC shims — the API was renamed between core 2.x and 3.x
// ------------------------------------------------------------
static inline void _pwmAttach(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(pin, freq, bits, ch);
#else
    ledcSetup(ch, freq, bits);
    ledcAttachPin(pin, ch);
#endif
}

static inline void _pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    (void)ch;   ledcWrite(pin,  duty);   // 3.x addresses the pin
#else
    (void)pin;  ledcWrite(ch,   duty);   // 2.x addresses the channel
#endif
}

// ------------------------------------------------------------
//  State
// ------------------------------------------------------------
static float    _yawDeg    = SERVO_YAW_HOME_DEG;
static float    _zoomDeg   = SERVO_ZOOM_HOME_DEG;
static bool     _torchOn   = false;
static uint8_t  _torchPct  = TORCH_BOOT_PCT;
static uint32_t _lastServoMs = 0;
static uint32_t _lastMoveMs  = 0;
static bool     _servoLive   = false;

// ------------------------------------------------------------
//  Hardware writes
// ------------------------------------------------------------
// 0..180 deg -> pulse width -> duty counts. At 50 Hz the frame is
// 20000 us, so one microsecond is (2^bits / 20000) counts.
static void _servoWriteDeg(uint8_t pin, uint8_t ch, float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float us = P.servoUsMin + (P.servoUsMax - P.servoUsMin) * (deg / 180.0f);
    const float countsPerUs = (float)(1UL << SERVO_PWM_BITS) / (1000000.0f / SERVO_PWM_HZ);
    _pwmWrite(pin, ch, (uint32_t)(us * countsPerUs + 0.5f));
}

// Every torch change in the whole firmware funnels through here.
static void _torchApply() {
    uint8_t pct = _torchOn ? _torchPct : 0;
    if (pct > TORCH_MAX_PCT) pct = TORCH_MAX_PCT;      // the hard cap
    uint32_t full = (1UL << TORCH_PWM_BITS) - 1UL;
    _pwmWrite(TORCH_PIN, TORCH_CH, (full * (uint32_t)pct) / 100UL);
}

// ------------------------------------------------------------
//  Public API
// ------------------------------------------------------------
// GPIO 12 is MTDI and GPIO 0 is the boot pin. After reset the ROM can
// leave a strapping pin attached to a non-GPIO peripheral function, and
// ledcAttachPin() will then appear to succeed while driving nothing.
// gpio_reset_pin() forces the pad back to plain GPIO before we claim it.
// The internal pullup it enables is then cleared, because a pullup on
// GPIO 12 is exactly what must NOT be present at the next reset.
static void _claimPin(uint8_t pin) {
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

inline void payload_begin() {
    _claimPin(TORCH_PIN);
    _claimPin(SERVO_YAW_PIN);
    _claimPin(SERVO_ZOOM_PIN);

    // Torch first, and explicitly off, so the gate is driven low
    // the moment the pin stops being a plain input.
    _pwmAttach(TORCH_PIN, TORCH_CH, TORCH_PWM_HZ, TORCH_PWM_BITS);
    _torchOn = false;
    _torchApply();

    _pwmAttach(SERVO_YAW_PIN,  SERVO_YAW_CH,  SERVO_PWM_HZ, SERVO_PWM_BITS);
    _pwmAttach(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, SERVO_PWM_HZ, SERVO_PWM_BITS);
    _yawDeg  = P.svYawHome;
    _zoomDeg = P.svZoomHome;
    _servoWriteDeg(SERVO_YAW_PIN,  SERVO_YAW_CH,  _yawDeg);
    _servoWriteDeg(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, _zoomDeg);
    _servoLive  = true;
    _lastMoveMs = millis();

    Serial.printf("[PAY] Servos yaw GPIO%d / zoom GPIO%d homed to %.0f / %.0f deg\n",
                  SERVO_YAW_PIN, SERVO_ZOOM_PIN,
                  (float)P.svYawHome, (float)P.svZoomHome);
    Serial.printf("[PAY] Torch GPIO%d off, %d%% preset, HARD CAP %d%% duty @ %d kHz\n",
                  TORCH_PIN, P.torchBoot, TORCH_MAX_PCT, TORCH_PWM_HZ / 1000);

    // Diagnostic: sample the zoom pin as an input before LEDC took it.
    // GPIO 12 must read LOW at reset or the chip picks the wrong flash
    // voltage. HIGH here means something on that line needs a pulldown.
    pinMode(SERVO_ZOOM_PIN, INPUT);
    int z = digitalRead(SERVO_ZOOM_PIN);
    Serial.printf("[PAY] GPIO%d resting level at boot: %s%s\n", SERVO_ZOOM_PIN,
                  z ? "HIGH" : "LOW",
                  z ? "  <-- fit a 10k resistor from this pin to GND" : "  (correct)");
    _claimPin(SERVO_ZOOM_PIN);
    _pwmAttach(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, SERVO_PWM_HZ, SERVO_PWM_BITS);
    _servoWriteDeg(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, _zoomDeg);
}

inline void payload_torchToggle() {
    _torchOn = !_torchOn;
    _torchApply();
    Serial.printf("[PAY] Torch %s (%d%%)\n", _torchOn ? "ON" : "off", _torchPct);
}

inline void payload_torchOff() {
    if (!_torchOn) return;
    _torchOn = false;
    _torchApply();
    Serial.println("[PAY] Torch off");
}

// steps of P.torchStep, clamped into P.torchMin..TORCH_MAX_PCT.
// Dimming while the torch is off just moves the preset silently.
inline void payload_torchStep(int8_t steps) {
    int v = (int)_torchPct + (int)steps * (int)P.torchStep;
    if (v < P.torchMin) v = P.torchMin;
    if (v > TORCH_MAX_PCT) v = TORCH_MAX_PCT;
    if ((uint8_t)v == _torchPct) return;
    _torchPct = (uint8_t)v;
    _torchApply();
    if (_torchOn) Serial.printf("[PAY] Torch %d%%%s\n", _torchPct,
                                _torchPct >= TORCH_MAX_PCT ? "  (cap)" : "");
}

inline void payload_torchSetPct(int pct) {
    if (pct < P.torchMin) pct = P.torchMin;
    if (pct > TORCH_MAX_PCT) pct = TORCH_MAX_PCT;
    _torchPct = (uint8_t)pct;
    _torchApply();
    Serial.printf("[PAY] Torch %d%% (%s)\n", _torchPct, _torchOn ? "on" : "off");
}

inline void payload_setYaw(float deg)  { _yawDeg  = constrain(deg, (float)P.svYawMin,  (float)P.svYawMax);  _lastMoveMs = millis(); }
inline void payload_setZoom(float deg) { _zoomDeg = constrain(deg, (float)P.svZoomMin, (float)P.svZoomMax); _lastMoveMs = millis(); }

inline void payload_center() {
    payload_setYaw(P.svYawHome);
    payload_setZoom(P.svZoomHome);
    Serial.println("[PAY] Servos re-centred");
}

inline bool  payload_torchIsOn() { return _torchOn;  }
inline uint8_t payload_torchPct(){ return _torchPct; }
inline float payload_yaw()       { return _yawDeg;   }
inline float payload_zoom()      { return _zoomDeg;  }

// Call once per control loop with the deadzoned stick values.
// Rate control, not absolute: stick deflection is a SPEED, so the
// camera stays where you left it when the spring-loaded stick
// re-centres. Absolute mapping would snap the zoom back to the
// middle every time you let go, which is useless for a lens.
inline void payload_update(float dt, float yawStick, float zoomStick) {

    bool moving = (yawStick != 0.0f) || (zoomStick != 0.0f);

    if (moving) {
        _yawDeg  += yawStick  * P.servoYawR  * dt;
        _zoomDeg += zoomStick * P.servoZoomR * dt;
        _yawDeg  = constrain(_yawDeg,  (float)P.svYawMin,  (float)P.svYawMax);
        _zoomDeg = constrain(_zoomDeg, (float)P.svZoomMin, (float)P.svZoomMax);
        _lastMoveMs = millis();
    }

    uint32_t now = millis();
    if (now - _lastServoMs < (uint32_t)(1000.0f / (P.svRate < 1 ? 1 : P.svRate))) return;   // one servo frame
    _lastServoMs = now;

    // Stop pulsing after a spell of no input so the MG90S stops hunting and
    // buzzing; it holds position on its own gearing. Runtime-gated now
    // (SVIDLE = 0 disables), so it can be toggled from the app.
    if (P.svIdleMs > 0 && !moving && (now - _lastMoveMs) > (uint32_t)P.svIdleMs) {
        if (_servoLive) {
            _pwmWrite(SERVO_YAW_PIN,  SERVO_YAW_CH,  0);
            _pwmWrite(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, 0);
            _servoLive = false;
        }
        return;
    }

    _servoLive = true;
    _servoWriteDeg(SERVO_YAW_PIN,  SERVO_YAW_CH,  _yawDeg);
    _servoWriteDeg(SERVO_ZOOM_PIN, SERVO_ZOOM_CH, _zoomDeg);
}

#endif // PAYLOAD_H
