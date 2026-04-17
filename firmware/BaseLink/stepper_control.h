/*
 * ============================================================
 *  stepper_control.h — Dual TMC2208 Stepper Driver Array
 * ============================================================
 *  Manages two NEMA17 stepper motors utilizing TMC2208 drivers
 *  operating natively in UART mode. 
 *  
 *  Step generation is entirely decoupled from the main process 
 *  loop via a dedicated high-frequency hardware timer Interrupt 
 *  Service Routine (ISR). This guarantees absolute timing
 *  precision and completely eliminates mechanical jitter.
 *
 *  A Bresenham-style pulse accumulator algorithm translates 
 *  continuous velocity demands into discrete, evenly distributed 
 *  step pulses across the timer's constant underlying frequency.
 * ============================================================
 */

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "config.h"

class StepperControl {
public:
    StepperControl();

    // ============================================================
    //  Initialization
    // ============================================================
    // Bootstraps UART communications, verifies driver register availability,
    // configures stealthChop parameters, and establishes the hardware timer ISR.
    // Returns gracefully as false if either TMC2208 integrated circuit fails 
    // to respond to a boot-up connectivity ping.
    bool begin();

    // ============================================================
    //  Kinetic Outputs
    // ============================================================
    // Applies an identical velocity vector unconditionally to both motors.
    // Parameter provided in steps per second (negative dictates reverse rotation).
    void setSpeed(int32_t stepsPerSec);

    // Applies independent velocity vectors, essential for executing coordinated
    // differential steering maneuvers while actively maintaining forward balance.
    void setSpeeds(int32_t leftStepsPerSec, int32_t rightStepsPerSec);

    // ============================================================
    //  Odometry & Position Tracking
    // ============================================================
    // The ISR autonomously increments/decrements these precise counters 
    // exactly simultaneously with every physical pulse dispatched.
    // This provides a pristine, drift-free methodology for tracking spatial 
    // displacement utilized exclusively by the Position-Hold algorithmic cascade.
    int32_t getPositionL() const;
    int32_t getPositionR() const;
    int32_t getAveragePosition() const;

    // ============================================================
    //  Driver States
    // ============================================================
    // Triggers the hardware EN (Enable) pin LOW, physically energizing the motor coils.
    void enable();

    // Triggers the hardware EN pin HIGH, physically de-energizing the coils, 
    // allowing the wheels to coast freely and safely.
    void disable();

    // Validates the logical software state tracking physical coil energization.
    bool isEnabled() const { return _enabled; }

    // ============================================================
    //  Interrupt Service Routine (ISR) Hook
    // ============================================================
    // The IRAM_ATTR directive strictly forces this function into localized 
    // instruction RAM, vastly accelerating execution times and preventing 
    // devastating cache miss latencies during the critical 20kHz interrupt.
    void IRAM_ATTR tick();

    // Dynamically alters the mathematical RMS current target sent directly to 
    // the driver chips via UART without necessitating a hard reboot.
    void setCurrent(uint16_t mA);

private:
    // Independent library abstraction objects managing UART payload formatting.
    TMC2208Stepper _rightDrv;
    TMC2208Stepper _leftDrv;

    // Direct pointer to the underlying ESP32 silicon hardware timer struct.
    hw_timer_t* _timer;

    // ============================================================
    //  ISR-Shared Volatile State
    // ============================================================
    // These variables traverse the boundary between the mainline processing core 
    // and the background hardware interrupt sequence. The volatile keyword 
    // strictly forbids the compiler from caching these values in isolated CPU registers.

    volatile uint32_t _absRateL;    // The absolute discrete velocity demand (Left).
    volatile uint32_t _absRateR;    // The absolute discrete velocity demand (Right).
    volatile bool     _dirFwdL;     // The finalized direction parity (Left).
    volatile bool     _dirFwdR;     // The finalized direction parity (Right).
    
    volatile uint32_t _accumR;      // Internal fractional accumulator (Bresenham Right).
    volatile uint32_t _accumL;      // Internal fractional accumulator (Bresenham Left).

    volatile int32_t  _posL;        // Absolute pulse tracker establishing odometry.
    volatile int32_t  _posR;        // Absolute pulse tracker establishing odometry.

    bool _enabled;                  // Mirrors the physical assertion state of the EN pins.

    // Internal abstraction handling monotonous boilerplate register configuration.
    void setupDriver(TMC2208Stepper& drv, const char* label);
};

// Global instance declaration allows the free-floating C-style ISR trampoline 
// function to access the heavily encapsulated class methods.
extern StepperControl steppers;

#endif // STEPPER_CONTROL_H
