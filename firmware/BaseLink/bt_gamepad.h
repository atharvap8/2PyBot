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
 *  CONTROLS
 *    Left stick Y   drive            Right stick X  steer
 *    START          arm              SELECT         disarm (E-STOP)
 *    Y / B / X / A  nod-yes / nod-no / spin / DANCE
 *    LB             toggle stiff hold
 * ============================================================
 */

#ifndef BT_GAMEPAD_H
#define BT_GAMEPAD_H

#include <Bluepad32.h>
#include "config.h"

// ---- same interface as espnow_comm.h ----
volatile float         joyForward      = 0.0f;   // -5..+5 legacy units
volatile float         joySteering     = 0.0f;   // -1..+1
volatile uint8_t       joyEnable       = 0;
volatile unsigned long lastJoyPacketMs = 0;

// ---- Bluepad32 bitmasks (verify with the [PAD] debug print) ----
#define PAD_BTN_A        0x0001
#define PAD_BTN_B        0x0002
#define PAD_BTN_X        0x0004
#define PAD_BTN_Y        0x0008
#define PAD_BTN_LB       0x0010
#define PAD_MISC_SELECT  0x0002
#define PAD_MISC_START   0x0004

static ControllerPtr _pad      = nullptr;
static bool          _armed    = false;
static uint16_t      _btnPrev  = 0;
static uint16_t      _miscPrev = 0;
static uint8_t       _gestReq  = 0;      // 1 yes, 2 no, 3 spin, 4 dance, 5 stiff-toggle

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

// Call once per control loop. Cheap: drains the BT event queue and maps.
inline void btgamepad_update() {
    BP32.update();

    if (!btgamepad_connected()) {
        joyForward = 0.0f;
        joySteering = 0.0f;
        // lastJoyPacketMs intentionally NOT refreshed -> goes stale ->
        // arbitration zeroes inputs, robot holds position. joyEnable
        // keeps its state so no disable edge fires (no surprise fall).
        return;
    }

    // Axes: Bluepad32 range -511..512, stick UP = negative Y.
    float fwdNorm   = _dz(-(float)_pad->axisY()  / 512.0f) * PAD_FWD_SIGN;
    float steerNorm = _dz( (float)_pad->axisRX() / 512.0f) * PAD_STEER_SIGN;

    joyForward  = -5.0f * fwdNorm;   // legacy units; .ino scales by -0.2
    joySteering = steerNorm;

    // ---- button edges ----
    uint16_t btn  = _pad->buttons();
    uint16_t misc = _pad->miscButtons();
    uint16_t bNew = btn  & ~_btnPrev;
    uint16_t mNew = misc & ~_miscPrev;

    if (mNew & PAD_MISC_START)  { _armed = true;  Serial.println("[PAD] ARM"); }
    if (mNew & PAD_MISC_SELECT) { _armed = false; Serial.println("[PAD] DISARM (e-stop)"); }

    if (bNew & PAD_BTN_Y)  _gestReq = 1;   // nod yes
    if (bNew & PAD_BTN_B)  _gestReq = 2;   // nod no
    if (bNew & PAD_BTN_X)  _gestReq = 3;   // spin
    if (bNew & PAD_BTN_A)  _gestReq = 4;   // dance!
    if (bNew & PAD_BTN_LB) _gestReq = 5;   // stiff hold toggle

    if (btn != _btnPrev || misc != _miscPrev)
        Serial.printf("[PAD] btn=0x%04X misc=0x%04X\n", btn, misc);  // for remapping

    _btnPrev  = btn;
    _miscPrev = misc;

    joyEnable       = _armed ? 1 : 0;
    lastJoyPacketMs = millis();
}

#endif // BT_GAMEPAD_H
