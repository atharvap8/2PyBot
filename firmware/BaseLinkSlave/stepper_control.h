/*
 * ============================================================
 *  stepper_control.h — Dual TMC2208 Stepper Driver
 * ============================================================
 *  Controls two NEMA17 steppers via TMC2208 in UART mode.
 *
 *  Step generation runs entirely inside a hardware timer ISR,
 *  decoupled from the main loop. A Bresenham accumulator
 *  converts continuous velocity demands into evenly distributed
 *  step pulses at the timer's fixed interrupt rate.
 *
 *  Odometry uses MT6816 magnetic encoders decoded by the ESP32
 *  PCNT hardware in 4x quadrature mode — not step counting.
 * ============================================================
 */

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "driver/pcnt.h"
#include "config.h"

class StepperControl {
public:
    // PCNT overflow accumulators — public so the C-style ISR
    // trampoline can write to them directly.
    static volatile int64_t _encOverflowL;
    static volatile int64_t _encOverflowR;

    StepperControl();

    // Configures GPIO, UART, TMC2208 registers, PCNT quadrature
    // decoding, and starts the hardware timer ISR.
    // Returns false if either TMC2208 does not respond over UART.
    bool begin();

    // Sets both motors to the same speed (steps/s, negative = reverse).
    void setSpeed(int32_t stepsPerSec);

    // Sets independent speeds for differential steering.
    void setSpeeds(int32_t leftStepsPerSec, int32_t rightStepsPerSec);

    // ---- Encoder-based odometry ----
    // Returns absolute shaft position from PCNT hardware (counts).
    // 4096 counts per revolution at 1024 PPR with 4x decode.
    int64_t getPositionL();
    int64_t getPositionR();
    int64_t getAveragePosition();

    // Pulls EN pin LOW — energizes motor coils.
    void enable();

    // Pulls EN pin HIGH — de-energizes coils, wheels coast freely.
    void disable();

    bool isEnabled() const { return _enabled; }

    // ISR tick handler — must be in IRAM for 20 kHz performance.
    void IRAM_ATTR tick();

    // Updates TMC2208 RMS current via UART without a reboot.
    void setCurrent(uint16_t mA);

private:
    TMC2208Stepper _rightDrv;
    TMC2208Stepper _leftDrv;
    hw_timer_t*    _timer;

    // Volatile ISR-shared state — compiler must not cache in registers.
    volatile uint32_t _absRateL;
    volatile uint32_t _absRateR;
    volatile bool     _dirFwdL;
    volatile bool     _dirFwdR;
    volatile uint32_t _accumR;
    volatile uint32_t _accumL;

    bool _enabled;

    void setupDriver(TMC2208Stepper& drv, const char* label);
    void setupPCNT(pcnt_unit_t unit, int pinA, int pinB);
};

// Global instance — accessed by the C-style ISR trampoline.
extern StepperControl steppers;

#endif // STEPPER_CONTROL_H
