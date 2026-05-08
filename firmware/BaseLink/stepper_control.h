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
 *
 *  ODOMETRY: Position feedback is sourced exclusively from 
 *  MT6816 magnetic encoders via ESP32 PCNT hardware quadrature 
 *  decoding — NOT from internal step counting. This provides 
 *  genuine closed-loop feedback immune to missed steps.
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
    // PCNT encoder overflow accumulators — public so the free C-style 
    // overflow ISR trampoline can accumulate counts directly.
    static volatile int64_t _encOverflowL;
    static volatile int64_t _encOverflowR;

    StepperControl();

    // ============================================================
    //  Initialization
    // ============================================================
    // Bootstraps UART communications, verifies driver register availability,
    // configures stealthChop parameters, establishes the hardware timer ISR,
    // and initializes PCNT hardware units for encoder feedback.
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
    //  Odometry & Position Tracking (ENCODER-BASED)
    // ============================================================
    // Reads real-time wheel positions directly from the MT6816 magnetic 
    // encoders via hardware PCNT quadrature decoding. These values represent 
    // actual physical shaft rotation — immune to stepper missed-step errors.
    // Units: encoder counts (4096 per revolution at 1024 PPR with 4x decode).
    int64_t getPositionL();
    int64_t getPositionR();
    int64_t getAveragePosition();

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

    bool _enabled;                  // Mirrors the physical assertion state of the EN pins.

    // Internal abstraction handling monotonous boilerplate register configuration.
    void setupDriver(TMC2208Stepper& drv, const char* label);
    
    // Configures one PCNT hardware unit for full 4x quadrature decoding.
    void setupPCNT(pcnt_unit_t unit, int pinA, int pinB);
};

// Global instance declaration allows the free-floating C-style ISR trampoline 
// function to access the heavily encapsulated class methods.
extern StepperControl steppers;

#endif // STEPPER_CONTROL_H
