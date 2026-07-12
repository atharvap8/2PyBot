/*
 * ============================================================
 *  config.h — EvoFox One S Bridge Configuration
 * ============================================================
 */

#ifndef EVOFOX_CONFIG_H
#define EVOFOX_CONFIG_H

// ============================================================
//  RECEIVER MAC ADDRESS
// ============================================================
// Must match the STA MAC printed by the robot on startup.
static uint8_t receiverMAC[] = {0xEC, 0x62, 0x60, 0x99, 0x97, 0xE0};

// ============================================================
//  HARDWARE
// ============================================================
#define STATUS_LED        2       // Onboard LED: solid = gamepad linked.

// ============================================================
//  CONTROL SHAPING
// ============================================================
#define MAX_TARGET_ANGLE  5.0f    // Max lean command (degrees).
#define MAX_STEERING      1.0f    // Max steering multiplier.

#define STICK_DEADZONE    0.08f   // Fractional center deadzone.
#define STICK_EXPO        0.35f   // 0 = linear, 1 = full cubic expo.
#define STICK_SMOOTH      0.6f    // EMA coefficient (higher = smoother).

// ============================================================
//  TRANSMIT RATE
// ============================================================
#define SEND_RATE_HZ      50
#define SEND_INTERVAL     (1000 / SEND_RATE_HZ)

#endif // EVOFOX_CONFIG_H
