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

static void _onPadConnect(ControllerPtr ctl) {
    _pad = ctl;
    Serial.printf("[PAD] Connected: %s\n", ctl->getModelName().c_str());
}
static void _onPadDisconnect(ControllerPtr ctl) {
    if (_pad == ctl) _pad = nullptr;
    Serial.println("[PAD] Disconnected — inputs zeroed, balance continues");
}

inline bool btgamepad_connected() { return _pad && _pad->isConnected(); }

inline void btgamepad_begin() {
    BP32.setup(&_onPadConnect, &_onPadDisconnect);
    // BP32.forgetBluetoothKeys();  // uncomment for ONE flash if pairing misbehaves
    Serial.println("[PAD] Bluepad32 ready — put the EVOFOX One S in pairing mode (Home+B)");
}

static inline float _dz(float v) { return (fabsf(v) < PAD_DEADZONE) ? 0.0f : v; }

// Returns the pending gesture request once, then clears it.
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

    // Axes: Bluepad32 range -511..512, stick UP = negative Y.
    float fwdNorm   = _dz(-(float)_pad->axisY() / 512.0f) * PAD_FWD_SIGN;
#if PAD_STEER_ON_LEFT_STICK
    float steerNorm = _dz( (float)_pad->axisX()  / 512.0f) * PAD_STEER_SIGN;
    joyPanX         = _dz( (float)_pad->axisRX() / 512.0f) * PAD_YAW_SIGN;
#else
    float steerNorm = _dz( (float)_pad->axisRX() / 512.0f) * PAD_STEER_SIGN;
    joyPanX         = 0.0f;          // right stick X is steering in this mode
#endif
    joyZoomY        = _dz(-(float)_pad->axisRY() / 512.0f) * PAD_ZOOM_SIGN;

    joyForward  = -5.0f * fwdNorm;   // legacy units; .ino scales by -0.2
    joySteering = steerNorm;

    // ---- button edges ----
    uint16_t btn  = _pad->buttons();
    uint16_t misc = _pad->miscButtons();
    uint8_t  dpad = _pad->dpad();
    uint16_t bNew = btn  & ~_btnPrev;
    uint16_t mNew = misc & ~_miscPrev;
    uint8_t  dNew = dpad & ~_dpadPrev;

    if (mNew & PAD_MISC_START)  { _armed = true;  Serial.println("[PAD] ARM"); }
    if (mNew & PAD_MISC_SELECT) { _armed = false; Serial.println("[PAD] DISARM (e-stop)"); }

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
    if (dNew & PAD_DPAD_RIGHT) { _torchStepReq++; _dpadRepeatMs = nowMs + PAD_REPEAT_FIRST_MS; }
    if (dNew & PAD_DPAD_LEFT)  { _torchStepReq--; _dpadRepeatMs = nowMs + PAD_REPEAT_FIRST_MS; }
    if (dpad & (PAD_DPAD_LEFT | PAD_DPAD_RIGHT)) {
        if ((int32_t)(nowMs - _dpadRepeatMs) >= 0) {
            _torchStepReq += (dpad & PAD_DPAD_RIGHT) ? 1 : -1;
            _dpadRepeatMs  = nowMs + PAD_REPEAT_MS;
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
