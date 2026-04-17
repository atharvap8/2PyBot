/*
 * ============================================================
 *  stepper_control.cpp — Dual TMC2208 implementation
 * ============================================================
 *  Provides the executable logic coordinating high-frequency 
 *  hardware interrupts with asynchronous UART configuration 
 *  payloads to seamlessly operate dual stepper motors.
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
      _posL(0), _posR(0),
      _enabled(false)
{}

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
//  Position Analytics
// ============================================================
// Disconnects identical Mutex boundaries temporarily to extract cleanly
// formatted position integers previously disjointed across multi-byte boundaries.
int32_t StepperControl::getPositionL() const {
    int32_t p;
    portENTER_CRITICAL(&timerMux); p = _posL; portEXIT_CRITICAL(&timerMux);
    return p;
}
int32_t StepperControl::getPositionR() const {
    int32_t p;
    portENTER_CRITICAL(&timerMux); p = _posR; portEXIT_CRITICAL(&timerMux);
    return p;
}
int32_t StepperControl::getAveragePosition() const {
    int32_t l, r;
    portENTER_CRITICAL(&timerMux);
    l = _posL; r = _posR;
    portEXIT_CRITICAL(&timerMux);
    return (l + r) / 2;
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
            
            // Maintain absolute chronological tracking.
            if (_dirFwdR) _posR++; else _posR--;
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
            
            if (_dirFwdL) _posL++; else _posL--;
        }
    }
}
