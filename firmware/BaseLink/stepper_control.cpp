/*
 * ============================================================
 *  stepper_control.cpp — Dual TMC2208 implementation
 * ============================================================
 *  Implements high-frequency hardware timer ISR step generation
 *  and asynchronous UART configuration for both stepper motors.
 *
 *  Odometry sourced from MT6816 magnetic encoders via PCNT
 *  hardware quadrature decoding. Step counting is not used.
 * ============================================================
 */

#include "stepper_control.h"
#include "soc/gpio_struct.h"

// ============================================================
//  Global instance & ISR trampoline
// ============================================================
StepperControl steppers;

// Mutex preventing data corruption if the ISR fires while the
// main loop is writing to the shared volatile speed variables.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// The hardware timer requires a plain C function pointer.
// This trampoline forwards the interrupt into the C++ class.
void IRAM_ATTR stepperTimerISR() {
    steppers.tick();
}

// ============================================================
//  Static PCNT overflow accumulators
// ============================================================
volatile int64_t StepperControl::_encOverflowL = 0;
volatile int64_t StepperControl::_encOverflowR = 0;

// ============================================================
//  PCNT overflow ISR — extends 16-bit counters to 64-bit
// ============================================================
// Fires when the PCNT hardware counter hits ±PCNT_H/L_LIM.
// Accumulates the overflow into 64-bit software trackers.
static void IRAM_ATTR pcnt_overflow_isr(void *arg) {
    uint32_t status = 0;

    pcnt_get_event_status(PCNT_UNIT_0, &status);
    if (status & PCNT_EVT_H_LIM) StepperControl::_encOverflowL += PCNT_H_LIM;
    if (status & PCNT_EVT_L_LIM) StepperControl::_encOverflowL += PCNT_L_LIM;

    pcnt_get_event_status(PCNT_UNIT_1, &status);
    if (status & PCNT_EVT_H_LIM) StepperControl::_encOverflowR += PCNT_H_LIM;
    if (status & PCNT_EVT_L_LIM) StepperControl::_encOverflowR += PCNT_L_LIM;
}

// ============================================================
//  Constructor
// ============================================================
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
//  setupPCNT() — configures one PCNT unit for 4x quadrature
// ============================================================
// Channel 0: count on A edges, use B for direction.
// Channel 1: count on B edges, use A for direction.
// Together these produce 4x resolution (4096 counts/rev).
void StepperControl::setupPCNT(pcnt_unit_t unit, int pinA, int pinB) {
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

    // Glitch filter: ignore pulses shorter than 100 × 12.5 ns = 1.25 µs.
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// ============================================================
//  begin()
// ============================================================
bool StepperControl::begin() {

    // GPIO setup.
    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_EN_PIN,   OUTPUT);
    pinMode(LEFT_STEP_PIN,  OUTPUT);
    pinMode(LEFT_DIR_PIN,   OUTPUT);
    pinMode(LEFT_EN_PIN,    OUTPUT);

    // Disable motors immediately on boot.
    digitalWrite(RIGHT_EN_PIN, HIGH);
    digitalWrite(LEFT_EN_PIN,  HIGH);

    // Serial2 defaults to pins 16/17 (Right driver).
    // Serial1 is remapped to pins 18/19 (Left driver).
    Serial2.begin(115200, SERIAL_8N1, RIGHT_UART_RX, RIGHT_UART_TX);
    Serial1.begin(115200, SERIAL_8N1, LEFT_UART_RX,  LEFT_UART_TX);
    delay(100);  // Allow UART lines to settle before transmitting.

    setupDriver(_rightDrv, "RIGHT");
    setupDriver(_leftDrv,  "LEFT");

    // PCNT_UNIT_0 = Left wheel; PCNT_UNIT_1 = Right wheel.
    setupPCNT(PCNT_UNIT_0, ENC_LEFT_A,  ENC_LEFT_B);
    setupPCNT(PCNT_UNIT_1, ENC_RIGHT_A, ENC_RIGHT_B);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_overflow_isr, NULL);
    pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_overflow_isr, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_intr_enable(PCNT_UNIT_1);

    Serial.println("[STEP] MT6816 encoders initialized (PCNT 4x decode, 4096 CPR)");

    // Timer at 1 MHz tick; TIMER_ALARM_COUNT sets the interrupt frequency.
    // The API changed between Arduino-ESP32 cores: 3.x is frequency-based,
    // the 2.x core bundled with the Bluepad32 board package is divider-based.
    // Both paths produce the identical 1 MHz tick and 20 kHz alarm.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    _timer = timerBegin(1000000);
    timerAttachInterrupt(_timer, &stepperTimerISR);
    timerAlarm(_timer, TIMER_ALARM_COUNT, true, 0);
#else
    _timer = timerBegin(0, TIMER_PRESCALER, true);       // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(_timer, &stepperTimerISR, true);
    timerAlarmWrite(_timer, TIMER_ALARM_COUNT, true);    // auto-reload
    timerAlarmEnable(_timer);
#endif

    Serial.println("[STEP] Timer started");
    return true;
}

// ============================================================
//  setupDriver()
// ============================================================
// Writes operational parameters to a TMC2208 via UART.
void StepperControl::setupDriver(TMC2208Stepper& drv, const char* label) {
    drv.begin();
    drv.toff(5);
    drv.rms_current(MOTOR_CURRENT_MA);
    drv.microsteps(MICROSTEPS);
    drv.pwm_autoscale(true);
    drv.en_spreadCycle(true);

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
void StepperControl::setSpeed(int32_t stepsPerSec) {
    setSpeeds(stepsPerSec, stepsPerSec);
}

void StepperControl::setSpeeds(int32_t leftStepsPerSec, int32_t rightStepsPerSec) {
    // Clamp to timer frequency — more steps per second than interrupts is impossible.
    if (leftStepsPerSec  >  (int32_t)TIMER_FREQ_HZ) leftStepsPerSec  =  TIMER_FREQ_HZ;
    if (leftStepsPerSec  < -(int32_t)TIMER_FREQ_HZ) leftStepsPerSec  = -TIMER_FREQ_HZ;
    if (rightStepsPerSec >  (int32_t)TIMER_FREQ_HZ) rightStepsPerSec =  TIMER_FREQ_HZ;
    if (rightStepsPerSec < -(int32_t)TIMER_FREQ_HZ) rightStepsPerSec = -TIMER_FREQ_HZ;

    bool fwdL = (leftStepsPerSec  >= 0);
    bool fwdR = (rightStepsPerSec >= 0);
    uint32_t rateL = (uint32_t)abs(leftStepsPerSec);
    uint32_t rateR = (uint32_t)abs(rightStepsPerSec);

    // Set DIR pins outside the ISR to keep interrupt latency minimal.
    bool pinDirL = fwdL ^ LEFT_DIR_INVERT;
    bool pinDirR = fwdR ^ RIGHT_DIR_INVERT;
    digitalWrite(LEFT_DIR_PIN,  pinDirL ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, pinDirR ? HIGH : LOW);

    // Atomic update of shared volatile variables.
    portENTER_CRITICAL_ISR(&timerMux);
    _dirFwdL  = fwdL;
    _dirFwdR  = fwdR;
    _absRateL = rateL;
    _absRateR = rateR;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================
//  Encoder position reads (PCNT-based)
// ============================================================
int64_t StepperControl::getPositionL() {
    int16_t hw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &hw);
    return _encOverflowL + hw;
}

int64_t StepperControl::getPositionR() {
    int16_t hw = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &hw);
    return -(_encOverflowR + hw);  // Inverted to match left encoder's forward direction.
}

int64_t StepperControl::getAveragePosition() {
    return (getPositionL() + getPositionR()) / 2;
}

// ============================================================
//  enable() / disable()
// ============================================================
void StepperControl::enable() {
    digitalWrite(RIGHT_EN_PIN, LOW);
    digitalWrite(LEFT_EN_PIN,  LOW);
    _enabled = true;
    Serial.println("[STEP] Motors ENABLED");
}

void StepperControl::disable() {
    // Zero ISR velocities before de-energizing to prevent coasting steps.
    portENTER_CRITICAL_ISR(&timerMux);
    _absRateL = 0;
    _absRateR = 0;
    _accumR   = 0;
    _accumL   = 0;
    portEXIT_CRITICAL_ISR(&timerMux);

    digitalWrite(RIGHT_EN_PIN, HIGH);
    digitalWrite(LEFT_EN_PIN,  HIGH);
    _enabled = false;
    Serial.println("[STEP] Motors DISABLED");
}

// ============================================================
//  setCurrent()
// ============================================================
void StepperControl::setCurrent(uint16_t mA) {
    _rightDrv.rms_current(mA);
    _leftDrv.rms_current(mA);
    Serial.printf("[STEP] Motor current set to %d mA\n", mA);
}

// ============================================================
//  tick() — 20 kHz ISR (Bresenham pulse generator)
// ============================================================
// Each call accumulates the requested velocity into a pool.
// When the pool exceeds TIMER_FREQ_HZ, one step pulse is emitted
// and the threshold is subtracted. This distributes pulses evenly
// regardless of the speed value's relationship to the timer rate.
//
// GPIO is manipulated via direct silicon registers (out1_w1ts/out_w1ts)
// for single-cycle execution within the time-critical interrupt.
void IRAM_ATTR StepperControl::tick() {

    // ---- Right motor ----
    if (_absRateR > 0) {
        _accumR += _absRateR;
        if (_accumR >= TIMER_FREQ_HZ) {
            _accumR -= TIMER_FREQ_HZ;
            GPIO.out1_w1ts.val = (1 << (RIGHT_STEP_PIN - 32));
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            GPIO.out1_w1tc.val = (1 << (RIGHT_STEP_PIN - 32));
        }
    }

    // ---- Left motor ----
    if (_absRateL > 0) {
        _accumL += _absRateL;
        if (_accumL >= TIMER_FREQ_HZ) {
            _accumL -= TIMER_FREQ_HZ;
            GPIO.out_w1ts = (1 << LEFT_STEP_PIN);
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
            GPIO.out_w1tc = (1 << LEFT_STEP_PIN);
        }
    }
}
