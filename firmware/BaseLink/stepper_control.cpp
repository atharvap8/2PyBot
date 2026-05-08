/*
 * ============================================================
 *  stepper_control.cpp — Dual TMC2208 implementation
 * ============================================================
 *  Provides the executable logic coordinating high-frequency 
 *  hardware interrupts with asynchronous UART configuration 
 *  payloads to seamlessly operate dual stepper motors.
 *
 *  ODOMETRY: All position feedback is sourced from MT6816 
 *  magnetic encoders via ESP32 PCNT hardware — replacing the 
 *  previous open-loop step counting entirely.
 * ============================================================
 */

#include "stepper_control.h"
#include "soc/gpio_struct.h"

// ============================================================
//  Global Instance & ISR Trampoline
// ============================================================
// Instantiates the primary operational motor control object globally.
StepperControl steppers;

// An RTOS-level Mutex (Mutual Exclusion) lock. This structure prevents
// catastrophic data corruption occurring if the background ISR interrupt
// executes precisely while the main foreground loop is in the middle
// of calculating and updating the identical shared variable array.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// A simplistic C-style function forcibly injected into the highest priority
// execution RAM. The underlying silicon timer mandates a standard C-pointer
// callback, hence this 'trampoline' structure simply intercepts the raw 
// hardware trigger and instantly redirects it back into our C++ class structure.
void IRAM_ATTR stepperTimerISR() {
    steppers.tick();
}

// ============================================================
//  Static PCNT overflow accumulators (class-level storage)
// ============================================================
volatile int64_t StepperControl::_encOverflowL = 0;
volatile int64_t StepperControl::_encOverflowR = 0;

// ============================================================
//  PCNT Overflow ISR — Extends 16-bit counters to 64-bit
// ============================================================
// When the hardware counter hits ±30000, this ISR fires and 
// accumulates the overflow into the 64-bit software tracker.
static void IRAM_ATTR pcnt_overflow_isr(void *arg) {
    uint32_t status = 0;

    // Left encoder (PCNT_UNIT_0)
    pcnt_get_event_status(PCNT_UNIT_0, &status);
    if (status & PCNT_EVT_H_LIM) StepperControl::_encOverflowL += PCNT_H_LIM;
    if (status & PCNT_EVT_L_LIM) StepperControl::_encOverflowL += PCNT_L_LIM;

    // Right encoder (PCNT_UNIT_1)
    pcnt_get_event_status(PCNT_UNIT_1, &status);
    if (status & PCNT_EVT_H_LIM) StepperControl::_encOverflowR += PCNT_H_LIM;
    if (status & PCNT_EVT_L_LIM) StepperControl::_encOverflowR += PCNT_L_LIM;
}

// ============================================================
//  Constructor
// ============================================================
// Initializes the serial driver abstractions. The TMC2208Stepper instances
// distinctly mandate a reference to an active Stream abstraction and the
// physical value of the sense resistor populated on the external PCB.
StepperControl::StepperControl()
    : _rightDrv(&Serial2, R_SENSE),
      _leftDrv (&Serial1, R_SENSE),
      _timer(nullptr),
      _absRateL(0), _absRateR(0), 
      _dirFwdL(true), _dirFwdR(true),
      _accumR(0), _accumL(0),
      _enabled(false)
{}

// ============================================================
//  setupPCNT() — Configure one PCNT unit for 4x quadrature
// ============================================================
// Uses both PCNT channels on a single unit to achieve full 4x 
// decode: counting on both A and B edges with direction from 
// the opposite signal.
void StepperControl::setupPCNT(pcnt_unit_t unit, int pinA, int pinB) {
    // Channel 0: count on A edges, use B for direction
    pcnt_config_t cfg0 = {};
    cfg0.pulse_gpio_num = pinA;
    cfg0.ctrl_gpio_num  = pinB;
    cfg0.channel         = PCNT_CHANNEL_0;
    cfg0.unit            = unit;
    cfg0.pos_mode        = PCNT_COUNT_INC;
    cfg0.neg_mode        = PCNT_COUNT_DEC;
    cfg0.lctrl_mode      = PCNT_MODE_REVERSE;
    cfg0.hctrl_mode      = PCNT_MODE_KEEP;
    cfg0.counter_h_lim   = PCNT_H_LIM;
    cfg0.counter_l_lim   = PCNT_L_LIM;
    pcnt_unit_config(&cfg0);

    // Channel 1: count on B edges, use A for direction (completes 4x)
    pcnt_config_t cfg1 = {};
    cfg1.pulse_gpio_num = pinB;
    cfg1.ctrl_gpio_num  = pinA;
    cfg1.channel         = PCNT_CHANNEL_1;
    cfg1.unit            = unit;
    cfg1.pos_mode        = PCNT_COUNT_DEC;
    cfg1.neg_mode        = PCNT_COUNT_INC;
    cfg1.lctrl_mode      = PCNT_MODE_REVERSE;
    cfg1.hctrl_mode      = PCNT_MODE_KEEP;
    cfg1.counter_h_lim   = PCNT_H_LIM;
    cfg1.counter_l_lim   = PCNT_L_LIM;
    pcnt_unit_config(&cfg1);

    // Glitch filter: ignore pulses shorter than 100 × 12.5 ns = 1.25 µs
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    // Enable overflow events
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    // Clear and start
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// ============================================================
//  begin()
// ============================================================
// Establishes physical constraints and instantiates the interrupt hierarchy.
bool StepperControl::begin() {
    
    // ---- Basic GPIO Construction ----
    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_EN_PIN,   OUTPUT);
    pinMode(LEFT_STEP_PIN,  OUTPUT);
    pinMode(LEFT_DIR_PIN,   OUTPUT);
    pinMode(LEFT_EN_PIN,    OUTPUT);

    // Actively pull enable pins HIGH immediately, firmly neutralizing motor coil currents.
    digitalWrite(RIGHT_EN_PIN, HIGH);
    digitalWrite(LEFT_EN_PIN,  HIGH);

    // ---- Hardware Serial Matrix Re-allocation ----
    // The ESP32 physically supports arbitrary pin routing for hardware interfaces.
    // Serial2 aligns directly to standard default layouts (Pins 16/17) servicing the Right driver.
    // Serial1 requires explicit programmatic remapping (Pins 18/19) servicing the Left driver.
    Serial2.begin(115200, SERIAL_8N1, RIGHT_UART_RX, RIGHT_UART_TX);
    Serial1.begin(115200, SERIAL_8N1, LEFT_UART_RX,  LEFT_UART_TX);

    // Enforce a minor chronological pause permitting voltages on the UART transit lines 
    // to strictly normalize before transmission.
    delay(100);

    // ---- Execute Register Configuration ----
    // Sequentially pump configuration data frames through the UART streams.
    setupDriver(_rightDrv, "RIGHT");
    setupDriver(_leftDrv,  "LEFT");

    // ---- MT6816 Encoder PCNT Initialization ----
    // Sets up hardware quadrature decoding for both wheel encoders.
    // PCNT_UNIT_0 = Left wheel, PCNT_UNIT_1 = Right wheel.
    setupPCNT(PCNT_UNIT_0, ENC_LEFT_A,  ENC_LEFT_B);
    setupPCNT(PCNT_UNIT_1, ENC_RIGHT_A, ENC_RIGHT_B);

    // Install shared overflow ISR for both PCNT units
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_overflow_isr, NULL);
    pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_overflow_isr, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_1);

    Serial.println("[STEP] MT6816 encoders initialized (PCNT 4x decode, 4096 CPR)");

    // ---- Hardware Timer Structure (ESP32 Core 3.0+ Compliant) ----
    // Requests exclusive access to an internal silicon timer.
    // Establishing the timer specifically at 1 MHz creates an intuitive 1-microsecond operational tick.
    _timer = timerBegin(1000000); 
    
    // Wire the physical timer event directly into our isolated execution trampoline.
    timerAttachInterrupt(_timer, &stepperTimerISR);
    
    // Formulate the alarm threshold. For instance, if the desired frequency is 20 kHz,
    // the system natively divides the 1 MHz tick by 20,000, configuring an interrupt every 50 ticks.
    // The 'true' parameter dictates the timer mathematically auto-reloads eternally.
    timerAlarm(_timer, TIMER_ALARM_COUNT, true, 0); 

    Serial.println("[STEP] Timer started");
    return true;
}

// ============================================================
//  setupDriver()
// ============================================================
// Evaluates and dictates complex operational states securely into the
// internal registers of the TMC2208 chip utilizing single-wire UART payloads.
void StepperControl::setupDriver(TMC2208Stepper& drv, const char* label) {
    drv.begin();                        // Initiates logical UART handshake procedures.
    drv.toff(5);                        // Establishes chopper off-time, requisite for operational continuity.
    drv.rms_current(MOTOR_CURRENT_MA);  // Overrides the primitive Vref potentiometer dynamically.
    drv.microsteps(MICROSTEPS);         // Scales structural resolution directly.
    drv.pwm_autoscale(true);            // Initializes the StealthChop2 PWM voltage auto-calibration sequence.
    drv.en_spreadCycle(true);           // Restricts operations exclusively within the ultra-quiet stealthChop paradigm.

    // Transmit an interrogation probe requesting fixed read-only identifiers back along the wire.
    // Any value aside from 0 explicitly signals a wire discontinuity or component failure.
    uint8_t conn = drv.test_connection();
    if (conn == 0) {
        Serial.printf("[STEP] %s TMC2208: OK  |  Current=%d mA  µsteps=%d\n",
                      label, MOTOR_CURRENT_MA, MICROSTEPS);
    } else {
        Serial.printf("[STEP] %s TMC2208: COMM ERROR (code %d) — check UART wiring\n",
                      label, conn);
    }
}

// ============================================================
//  setSpeed() / setSpeeds()
// ============================================================
// Modulates the autonomous ISR output velocities.
void StepperControl::setSpeed(int32_t stepsPerSec) {
    setSpeeds(stepsPerSec, stepsPerSec);
}

void StepperControl::setSpeeds(int32_t leftStepsPerSec, int32_t rightStepsPerSec) {
    // Statistically constrain mathematical boundaries absolutely prohibiting speeds 
    // exceeding the frequency of the interrupt executing them. Attempting to step 
    // twice within a single interrupt is physically impossible.
    if (leftStepsPerSec > (int32_t)TIMER_FREQ_HZ)  leftStepsPerSec =  TIMER_FREQ_HZ;
    if (leftStepsPerSec < -(int32_t)TIMER_FREQ_HZ) leftStepsPerSec = -TIMER_FREQ_HZ;
    if (rightStepsPerSec > (int32_t)TIMER_FREQ_HZ)  rightStepsPerSec =  TIMER_FREQ_HZ;
    if (rightStepsPerSec < -(int32_t)TIMER_FREQ_HZ) rightStepsPerSec = -TIMER_FREQ_HZ;

    // Abstract the directional intent separately from the definitive magnitude.
    bool fwdL = (leftStepsPerSec >= 0);
    bool fwdR = (rightStepsPerSec >= 0);
    
    uint32_t rateL = (uint32_t)abs(leftStepsPerSec);
    uint32_t rateR = (uint32_t)abs(rightStepsPerSec);

    // Apply mirroring corrections and physically modulate the digital DIR pins directly.
    // Operating pins outside the ISR ensures processing time within the interrupt is utterly minimized.
    bool pinDirL = fwdL ^ LEFT_DIR_INVERT;
    bool pinDirR = fwdR ^ RIGHT_DIR_INVERT;
    digitalWrite(LEFT_DIR_PIN,  pinDirL ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, pinDirR ? HIGH : LOW);

    // Suspend RTOS operations temporarily bridging the mutex gap to atomically 
    // deploy the updated metrics directly into the volatile execution space.
    portENTER_CRITICAL_ISR(&timerMux);
    _dirFwdL  = fwdL;
    _dirFwdR  = fwdR;
    _absRateL = rateL;
    _absRateR = rateR;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================
//  Position Analytics — REAL ENCODER FEEDBACK
// ============================================================
// Reads accumulated encoder counts from the PCNT hardware units.
// Each count represents 1/4096th of a shaft revolution.
// This replaces the previous open-loop step counting entirely.

int64_t StepperControl::getPositionL() {
    int16_t hw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &hw);
    return _encOverflowL + hw;
}

int64_t StepperControl::getPositionR() {
    int16_t hw = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &hw);
    return -(_encOverflowR + hw); // Inverted to match left encoder's forward direction
}

int64_t StepperControl::getAveragePosition() {
    return (getPositionL() + getPositionR()) / 2;
}

// ============================================================
//  Hardware Energization
// ============================================================
void StepperControl::enable() {
    digitalWrite(RIGHT_EN_PIN, LOW);
    digitalWrite(LEFT_EN_PIN,  LOW);
    _enabled = true;
    Serial.println("[STEP] Motors ENABLED");
}

void StepperControl::disable() {
    // Immediately suppress any active ISR algorithmic propagation prior 
    // to logically lifting the hardware constraints.
    portENTER_CRITICAL_ISR(&timerMux);
    _absRateL = 0;
    _absRateR = 0;
    _accumR  = 0;
    _accumL  = 0;
    portEXIT_CRITICAL_ISR(&timerMux);

    digitalWrite(RIGHT_EN_PIN, HIGH);
    digitalWrite(LEFT_EN_PIN,  HIGH);
    _enabled = false;
    Serial.println("[STEP] Motors DISABLED");
}

// ============================================================
//  Dynamic Configuration
// ============================================================
void StepperControl::setCurrent(uint16_t mA) {
    // Flushes updated current parameters asynchronously into the UART line.
    _rightDrv.rms_current(mA);
    _leftDrv.rms_current(mA);
    Serial.printf("[STEP] Motor current set to %d mA\n", mA);
}

// ============================================================
//  tick() — High-Frequency Execution Segment
// ============================================================
// Employs a fixed-point numerical adaptation of Bresenham's iconic algorithm.
// 
// Every sequence aggregates the requested velocity parameter (`_absRateR`) 
// persistently into an overarching pool (`_accumR`). 
//
// When the accumulated volume structurally breaches the boundary defined 
// by the total frequency (`TIMER_FREQ_HZ`), the system acknowledges a necessary 
// step event. The system disperses exactly one physical logic pulse immediately, 
// subsequently expunging the frequency boundary amount from the pool.
// 
// This creates flawlessly uniform pulse displacement, even mapping obscure or
// prime numerical velocity constraints optimally across constant procedural intervals.
//
// NOTE: Step counting has been REMOVED from this ISR. Position tracking is now 
// handled entirely by the MT6816 encoder PCNT hardware — true closed-loop feedback.

void IRAM_ATTR StepperControl::tick() {
    
    // ---- Right Motor Analysis ----
    if (_absRateR > 0) {
        _accumR += _absRateR;
        if (_accumR >= TIMER_FREQ_HZ) {
            _accumR -= TIMER_FREQ_HZ;
            
            // Replaces convoluted Arduino abstraction (digitalWrite) entirely with
            // pure native Espressif architectural silicon manipulation.
            // Writing definitively to out1_w1ts forces the requisite pin HIGH in a single clock cycle.
            GPIO.out1_w1ts.val = (1 << (RIGHT_STEP_PIN - 32));
            
            // Assembly 'NOP' instructions artificially widen the digital waveform.
            // Hardware stepper interfaces fundamentally require minimum identifiable pulse widths.
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            
            // Collapse the physical waveform back LOW.
            GPIO.out1_w1tc.val = (1 << (RIGHT_STEP_PIN - 32));
            
            // Position tracking removed — encoder handles this now.
        }
    }

    // ---- Left Motor Analysis ----
    if (_absRateL > 0) {
        _accumL += _absRateL;
        if (_accumL >= TIMER_FREQ_HZ) {
            _accumL -= TIMER_FREQ_HZ;
            
            // Silicon registers for pins entirely beneath ID 32.
            GPIO.out_w1ts = (1 << LEFT_STEP_PIN);
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            GPIO.out_w1tc = (1 << LEFT_STEP_PIN);
            
            // Position tracking removed — encoder handles this now.
        }
    }
}
