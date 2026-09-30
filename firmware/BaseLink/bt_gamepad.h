/*
 * ============================================================
 *  bt_gamepad.h — EVOFOX One S direct Bluetooth (Bluepad32)
 * ============================================================
 *  Drop-in replacement for espnow_comm.h. It exposes the SAME
 *  four globals (joyForward, joySteering, joyEnable,
 *  lastJoyPacketMs) with the SAME units and semantics, so the
 *  arbitration and enable-edge code in the .ino is unchanged.
 *
 *  joyForward is emitted in the legacy -5..+5 "pitch offset"
 *  units; the .ino still multiplies by JOY_FWD_SCALE (-0.2) to
 *  get the normalized forward command. Nothing downstream moved.
 *
 *  REQUIRES: the "esp32_bluepad32" board package selected in the
 *  Arduino IDE (see README_BT.md). BT stack runs on core 0; the
 *  balance loop and 20 kHz stepper ISR stay on core 1.
 *
 *  CONTROLS  (remapped for the camera payload)
 *    LEFT STICK  = the robot        RIGHT STICK = the camera
 *      Left  Y      drive             Right X      servo YAW / pan
 *      Left  X      steer             Right Y      servo ZOOM
 *    START          arm              SELECT        disarm (E-STOP)
 *    Y / B / X / A  nod-yes / nod-no / spin / DANCE
 *    LB             toggle SPEED LOW <-> HIGH
 *    RB             toggle FLASHLIGHT
 *    D-pad UP       toggle stiff hold   D-pad DOWN  toggle CLIMB mode
 *    D-pad LEFT     torch dimmer        D-pad RIGHT torch brighter
 *                   (both auto-repeat while held)
 *
 *  TWO DELIBERATE CHANGES vs the old map, both forced by the
 *  payload needing the whole right stick and the RB button:
 *    1. STEER MOVED to the LEFT stick X. One stick now does all
 *       driving, which frees the right stick entirely for the
 *       camera. Set PAD_STEER_ON_LEFT_STICK 0 in this file to put
 *       steering back on the right stick (yaw then loses its axis).
 *    2. SPEED is now an LB TOGGLE instead of LB=low / RB=high,
 *       because RB became the torch. Same two modes, one button.
 * ============================================================
 */

#ifndef BT_GAMEPAD_H
#define BT_GAMEPAD_H

#include <Bluepad32.h>
#include "config.h"
#include "params.h"

// 1 = steer on LEFT stick X (right stick free for the camera).
// 0 = legacy: steer on RIGHT stick X, and servo yaw goes dead.
#define PAD_STEER_ON_LEFT_STICK 1

// ---- same interface as espnow_comm.h ----
volatile float         joyForward      = 0.0f;   // -5..+5 legacy units
volatile float         joySteering     = 0.0f;   // -1..+1
volatile uint8_t       joyEnable       = 0;
volatile unsigned long lastJoyPacketMs = 0;

// ---- camera payload axes, -1..+1, deadzoned (right stick) ----
volatile float         joyPanX         = 0.0f;   // + = pan right
volatile float         joyZoomY        = 0.0f;   // + = stick forward

// Dual speed mode: 0 = LOW (LB), 1 = HIGH (RB). Consumed by the .ino
// arbitration; sticky across arm/disarm and pad reconnects on purpose
// (predictable — the mode you left is the mode you get back).
volatile uint8_t       joySpeedHigh    = SPEED_BOOT_HIGH;

// ---- Bluepad32 bitmasks (verify with the [PAD] debug print) ----
#define PAD_BTN_A        0x0001
#define PAD_BTN_B        0x0002
#define PAD_BTN_X        0x0004
#define PAD_BTN_Y        0x0008
#define PAD_BTN_LB       0x0010   // shoulder L
#define PAD_BTN_RB       0x0020   // shoulder R
#define PAD_BTN_LT       0x0040   // trigger L (digital bit)
#define PAD_BTN_RT       0x0080   // trigger R (digital bit)
#define PAD_MISC_SELECT  0x0002
#define PAD_MISC_START   0x0004
// D-pad comes from _pad->dpad(), a separate bitmask from buttons():
#define PAD_DPAD_UP      0x01
#define PAD_DPAD_DOWN    0x02
#define PAD_DPAD_RIGHT   0x04
#define PAD_DPAD_LEFT    0x08

static ControllerPtr _pad      = nullptr;
static bool          _armed    = false;
static uint16_t      _btnPrev  = 0;
static uint16_t      _miscPrev = 0;
static uint8_t       _dpadPrev = 0;
static uint8_t       _gestReq  = 0;      // 1 yes, 2 no, 3 spin, 4 dance, 5 stiff-toggle, 6 climb-toggle
static uint8_t       _torchToggleReq = 0;   // pending RB presses
static int8_t        _torchStepReq   = 0;   // net D-pad L/R clicks, signed
static uint32_t      _dpadRepeatMs   = 0;   // next auto-repeat due

// ---- stick centre calibration + first-frame guard ----
static float        _cx = 0.0f, _cy = 0.0f;    // left-stick centre offsets
static float        _crx = 0.0f, _cry = 0.0f;  // right-stick centre offsets
static bool         _calDone     = false;
static bool         _freshConnect = true;      // swallow edges on frame 1



static void _onPadConnect(ControllerPtr ctl) {
    _pad = ctl;
    _freshConnect = true;
    Serial.printf("[PAD] Connected: %s\n", ctl->getModelName().c_str());
}
static void _onPadDisconnect(ControllerPtr ctl) {
    _calDone = false;
    _freshConnect = true;
    if (_pad == ctl) _pad = nullptr;
    Serial.println("[PAD] Disconnected — inputs zeroed, balance continues");
}

inline bool btgamepad_connected() { return _pad && _pad->isConnected(); }

inline void btgamepad_begin() {
    BP32.setup(&_onPadConnect, &_onPadDisconnect);
    // BP32.forgetBluetoothKeys();  // uncomment for ONE flash if pairing misbehaves
    Serial.println("[PAD] Bluepad32 ready — put the EVOFOX One S in pairing mode (Home+B)");
}

static inline float _dz(float v) { return (fabsf(v) < P.padDz) ? 0.0f : v; }

// Returns the pending gesture request once, then clears it.
// +1 = START pressed, -1 = SELECT pressed, 0 = nothing. Clears on read.
// The .ino acts on these EVENTS, so SELECT e-stops even if START was
// never pressed (the old level-edge logic could not do that).
static int8_t _enableEvent = 0;
inline int8_t btgamepad_takeEnableEvent() { int8_t e=_enableEvent; _enableEvent=0; return e; }
inline void   btgamepad_syncArmed(bool a) { _armed = a; }

inline uint8_t btgamepad_takeGesture() {
    uint8_t g = _gestReq; _gestReq = 0; return g;
}

// True once per RB press (torch toggle). Coalesces if the loop
// ever falls behind: an odd number of presses is still one toggle.
inline bool btgamepad_takeTorchToggle() {
    bool t = (_torchToggleReq & 1); _torchToggleReq = 0; return t;
}

// Net brightness clicks since the last call: + brighter, - dimmer.
inline int8_t btgamepad_takeTorchStep() {
    int8_t s = _torchStepReq; _torchStepReq = 0; return s;
}

static inline float _smoothstep(float e0, float e1, float x) {
    float t = (x - e0) / (e1 - e0);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}
static inline float _expo(float v, float e) {
    float a = fabsf(v);
    return copysignf(e * a * a * a + (1.0f - e) * a, v);
}

// Radial deadzone + rescale + cardinal snap + expo.
// x = lateral axis, y = forward axis. Outputs written through pointers.
// snap != 0 applies the "protect straight ahead" taper to the x output.
static void _shapeStick(float x, float y, bool snap, float* outY, float* outX) {
    float r = sqrtf(x * x + y * y);
    if (r < P.stickDz) { *outY = 0.0f; *outX = 0.0f; return; }

    // rescale so output starts at 0 exactly at the deadzone edge (no jump)
    float k = ((r - P.stickDz) / (1.0f - P.stickDz)) / r;
    x *= k; y *= k;
    if (x >  1.0f) x =  1.0f;  if (x < -1.0f) x = -1.0f;
    if (y >  1.0f) y =  1.0f;  if (y < -1.0f) y = -1.0f;

    float w = 1.0f;
    if (snap) {
        // angle away from the pure fwd/back axis: 0 = straight, pi/2 = pure turn
        float ang = atan2f(fabsf(x), fabsf(y));
        w = _smoothstep(P.snapIn * (float)DEG_TO_RAD,
                        P.snapOut * (float)DEG_TO_RAD, ang);
    }
    *outY = _expo(y, P.expoDrive);
    *outX = _expo(x * w, P.expoSteer);
}

// Call once per control loop. Cheap: drains the BT event queue and maps.
inline void btgamepad_update() {
    BP32.update();

    if (!btgamepad_connected()) {
        joyForward = 0.0f;
        joySteering = 0.0f;
        joyPanX     = 0.0f;   // rate control -> servos simply hold still
        joyZoomY    = 0.0f;
        // lastJoyPacketMs intentionally NOT refreshed -> goes stale ->
        // arbitration zeroes inputs, robot holds position. joyEnable
        // keeps its state so no disable edge fires (no surprise fall).
        return;
    }

        // raw, centre-corrected, normalised. Bluepad32 is -511..512, up = -Y.
    float rawX  =  (float)_pad->axisX()  / 512.0f - _cx;
    float rawY  = -(float)_pad->axisY()  / 512.0f - _cy;
    float rawRX =  (float)_pad->axisRX() / 512.0f - _crx;
    float rawRY = -(float)_pad->axisRY() / 512.0f - _cry;

#if STICK_AUTOCAL
    // One-shot centre capture, only if the sticks are plausibly at rest.
    if (!_calDone) {
        if (fabsf(rawX) < 0.25f && fabsf(rawY) < 0.25f &&
            fabsf(rawRX) < 0.25f && fabsf(rawRY) < 0.25f) {
            _cx += rawX;  _cy += rawY;  _crx += rawRX; _cry += rawRY;
            _calDone = true;
            Serial.printf("[PAD] centre cal: L(%.3f,%.3f) R(%.3f,%.3f)\n",
                          _cx, _cy, _crx, _cry);
        }
        return;    // skip one frame; next loop uses the corrected centre
    }
#endif

    float fwdNorm = 0.0f, steerNorm = 0.0f, panNorm = 0.0f, zoomNorm = 0.0f;

#if PAD_STEER_ON_LEFT_STICK
    // LEFT stick does both -> snap protects straight ahead
    _shapeStick(rawX, rawY, true, &fwdNorm, &steerNorm);
    // RIGHT stick is the camera; snap keeps a pure pan from creeping the zoom
    _shapeStick(rawRX, rawRY, true, &zoomNorm, &panNorm);
    joyPanX  = panNorm  * P.padYawSgn;
    joyZoomY = zoomNorm * P.padZoomSgn;
#else
    // legacy: drive on left Y, steer on right X — separate sticks, no snap needed
    float dummy;
    _shapeStick(0.0f, rawY, false, &fwdNorm, &dummy);
    _shapeStick(rawRX, 0.0f, false, &dummy, &steerNorm);
    joyPanX  = 0.0f;
    _shapeStick(0.0f, rawRY, false, &zoomNorm, &dummy);
    joyZoomY = zoomNorm * P.padZoomSgn;
#endif

    fwdNorm   *= P.padFwdSgn;
    steerNorm *= P.padStrSgn;

    joyForward  = -5.0f * fwdNorm;   // legacy units; .ino scales by -0.2
    joySteering = steerNorm;

    // ---- button edges ----
    uint16_t btn  = _pad->buttons();
    uint16_t misc = _pad->miscButtons();
    uint8_t  dpad = _pad->dpad();
    if (_freshConnect) {
        // Adopt whatever is held at connect instead of treating it as a press.
        // Without this, connecting with Home held can ARM the robot by itself.
        _btnPrev = btn; _miscPrev = misc; _dpadPrev = dpad;
        _freshConnect = false;
        Serial.printf("[PAD] connect state adopted: btn=0x%04X misc=0x%04X dpad=0x%02X\n",
                      btn, misc, dpad);
        return;
    }
    uint16_t bNew = btn  & ~_btnPrev;
    uint16_t mNew = misc & ~_miscPrev;
    uint8_t  dNew = dpad & ~_dpadPrev;

    if (mNew & PAD_MISC_START)  { _armed = true;  _enableEvent = +1; Serial.println("[PAD] ARM"); }
    if (mNew & PAD_MISC_SELECT) { _armed = false; _enableEvent = -1; Serial.println("[PAD] DISARM (e-stop)"); }

    if (bNew & PAD_BTN_Y)    _gestReq = 1;   // nod yes
    if (bNew & PAD_BTN_B)    _gestReq = 2;   // nod no
    if (bNew & PAD_BTN_X)    _gestReq = 3;   // spin
    if (bNew & PAD_BTN_A)    _gestReq = 4;   // dance!
    if (dNew & PAD_DPAD_UP)   _gestReq = 5;   // stiff hold toggle
    if (dNew & PAD_DPAD_DOWN) _gestReq = 6;   // cliff/climb mode toggle

    // Dual speed mode — LB now TOGGLES, because RB became the torch.
    if (bNew & PAD_BTN_LB) {
        joySpeedHigh = joySpeedHigh ? 0 : 1;
        Serial.printf("[PAD] SPEED %s\n", joySpeedHigh ? "HIGH" : "LOW");
    }

    // RB toggles the flashlight. Consumed by the .ino, which owns
    // the payload; this file stays pure input mapping.
    if (bNew & PAD_BTN_RB) _torchToggleReq++;

    // D-pad LEFT/RIGHT trim torch brightness, with auto-repeat so you
    // can hold to sweep instead of tapping fifteen times.
    uint32_t nowMs = millis();
    if (dNew & PAD_DPAD_RIGHT) { _torchStepReq++; _dpadRepeatMs = nowMs + P.padRpt1; }
    if (dNew & PAD_DPAD_LEFT)  { _torchStepReq--; _dpadRepeatMs = nowMs + P.padRpt1; }
    if (dpad & (PAD_DPAD_LEFT | PAD_DPAD_RIGHT)) {
        if ((int32_t)(nowMs - _dpadRepeatMs) >= 0) {
            _torchStepReq += (dpad & PAD_DPAD_RIGHT) ? 1 : -1;
            _dpadRepeatMs  = nowMs + P.padRpt;
        }
    }

    if (btn != _btnPrev || misc != _miscPrev || dpad != _dpadPrev)
        Serial.printf("[PAD] btn=0x%04X misc=0x%04X dpad=0x%02X\n", btn, misc, dpad);  // for remapping

    _btnPrev  = btn;
    _miscPrev = misc;
    _dpadPrev = dpad;

    joyEnable       = _armed ? 1 : 0;
    lastJoyPacketMs = millis();
}

#endif // BT_GAMEPAD_H
