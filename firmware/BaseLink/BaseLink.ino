/*
 * ============================================================
 *  BaseLink_LQR.ino — 2PyBot firmware, Package B (LQR / LQI)
 * ============================================================
 *  Control law: optimal full-state feedback on the linearized
 *  velocity-driven inverted pendulum, augmented with a position
 *  integral (LQI) that nulls drift from the unknown CoG offset.
 *
 *      u = -( K1*(x-xRef) + K2*v + K3*pitch + K4*pitchRate + K5*z )
 *      z = Integral(x - xRef),   u = wheel acceleration
 *      vCmd += u*dt  ->  stepper ISR (steps/s)
 *
 *  Gains were computed offline by solving the continuous-time
 *  algebraic Riccati equation (see tools/simulate_controllers.py,
 *  which also regenerates them for different geometry). Validated
 *  in simulation: 0.5 cm RMS station keeping under a 0.35 deg CoG
 *  error, 2.3 deg peak excursion for a 45 deg/s shove, < 0.3 cm
 *  steady-state error on a 0.5 m position command.
 *
 *  Radxa serial bridge (USB serial, 115200):
 *    OUT 50 Hz :  O,<millis>,<encL>,<encR>,<pitchDeg>,<yawDeg>\n
 *    IN        :  V,<forward -1..1>,<steering -1..1>,<enable 0|1>\n
 *
 *  Tuner commands (same serial port, newline terminated):
 *    E enable    X e-stop    C cal gyro    S settings
 *    L toggle debug stream   R reset controller   ? help
 *    K1=..K5=<v> LQR gains   T=<v> pitch trim (deg)   M=<mA> current
 *
 *  ESP-NOW joystick keeps working as a fallback (Radxa has
 *  authority while its 'V' lines are fresh).
 * ============================================================
 */

#include "config.h"
#include "imu_sensor.h"
#include "stepper_control.h"
#include "espnow_comm.h"

IMUSensor imu;

// ============================================================
//  STATE
// ============================================================
enum RobotState { STATE_IDLE, STATE_BALANCING };
RobotState state       = STATE_IDLE;
bool motorsRequested   = false;
bool DEBUG_STREAM      = false;
bool RADXA_STREAM      = true;

// Forward-positive measurements (see config.h sign conventions)
float pitchF = 0.0f, rateF = 0.0f;      // deg, deg/sL
float posM   = 0.0f, velF  = 0.0f;      // m, m/s (encoder-based)
int64_t encRawL = 0, encRawR = 0;       // raw signed counts (for O-stream)
float encDiff = 0.0f, diffRateF = 0.0f; // (R - L) counts, fwd-signed

// Controller state
float vCmd = 0.0f;          // integrated velocity command (m/s)
float zInt = 0.0f;          // LQI position integral (m*s)
float xRef = 0.0f;          // position hold target (m)
float uOut = 0.0f;          // last acceleration command (m/s^2)
float steerSteps = 0.0f;    // current differential (steps/s)
float diffTarget = 0.0f;    // heading-hold encoder diff target

// Live-tunable gains (RAM copies of config defaults)
float k1 = LQR_K1, k2 = LQR_K2, k3 = LQR_K3, k4 = LQR_K4, k5 = LQR_K5;
float pitchTrim = PITCH_TRIM_DEG;

// Drive inputs after link arbitration
float cmdFwd = 0.0f, cmdSteer = 0.0f;

// Radxa link
uint32_t radxaLastRx = 0;
float    radxaFwd = 0.0f, radxaSteer = 0.0f;
uint8_t  radxaEn = 0, radxaEnPrev = 0;

// ESP-NOW edge tracking
uint8_t joyEnPrev = 0;

// Serial line buffer
char  rxBuf[96];
uint8_t rxLen = 0;

// Timing
unsigned long lastLoopUs = 0;
uint32_t lastOdomMs = 0, lastDbgMs = 0, lastBlinkMs = 0;
bool ledState = false;

// ============================================================
//  HELPERS
// ============================================================
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void resetController() {
    vCmd = 0.0f; zInt = 0.0f; uOut = 0.0f;
    xRef = posM;
    steerSteps = 0.0f; diffTarget = encDiff;
}

void requestEnable() {
    motorsRequested = true;
    Serial.println("[MAIN] Enable requested — will arm when upright");
}

void requestDisable(const char* why) {
    motorsRequested = false;
    if (state == STATE_BALANCING) {
        steppers.setSpeed(0);
        steppers.disable();
        state = STATE_IDLE;
    }
    Serial.printf("[MAIN] Motors OFF (%s)\n", why);
}

// ============================================================
//  SERIAL PARSER — Radxa 'V' lines + tuner commands, one reader
// ============================================================
void handleLine(char* line) {
    if (line[0] == 'V' && line[1] == ',') {
        float f = 0, s = 0; int en = 0;
        if (sscanf(line + 2, "%f,%f,%d", &f, &s, &en) == 3) {
            radxaFwd   = clampf(f, -1.0f, 1.0f);
            radxaSteer = clampf(s, -1.0f, 1.0f);
            radxaEn    = en ? 1 : 0;
            radxaLastRx = millis();
            if (radxaEn && !radxaEnPrev)  requestEnable();
            if (!radxaEn && radxaEnPrev)  requestDisable("Radxa disable");
            radxaEnPrev = radxaEn;
        }
        return;
    }

    // ---- tuner ----
    char cmd = toupper(line[0]);
    char* eq = strchr(line, '=');
    float val = eq ? atof(eq + 1) : 0.0f;

    if (eq) {
        if      (cmd == 'K' && line[1] == '1') { k1 = val; Serial.printf("[TUNE] K1 = %.4f\n", val); }
        else if (cmd == 'K' && line[1] == '2') { k2 = val; Serial.printf("[TUNE] K2 = %.4f\n", val); }
        else if (cmd == 'K' && line[1] == '3') { k3 = val; Serial.printf("[TUNE] K3 = %.4f\n", val); }
        else if (cmd == 'K' && line[1] == '4') { k4 = val; Serial.printf("[TUNE] K4 = %.4f\n", val); }
        else if (cmd == 'K' && line[1] == '5') { k5 = val; Serial.printf("[TUNE] K5 = %.4f\n", val); }
        else if (cmd == 'T') { pitchTrim = val; Serial.printf("[TUNE] Trim = %.3f deg\n", val); }
        else if (cmd == 'M') { steppers.setCurrent((uint16_t)val); }
        else Serial.printf("[TUNE] Unknown: %s\n", line);
        return;
    }

    switch (cmd) {
        case 'E': requestEnable(); break;
        case 'X': requestDisable("e-stop"); break;
        case 'C':
            requestDisable("gyro cal");
            imu.calibrateGyro();
            Serial.println("[TUNE] Send 'E' to re-arm.");
            break;
        case 'R': resetController(); Serial.println("[TUNE] Controller state reset"); break;
        case 'L': DEBUG_STREAM = !DEBUG_STREAM; break;
        case 'S':
            Serial.println("\n---- Package B (LQR / LQI) settings ----");
            Serial.printf("  K1=%.4f  K2=%.4f  K3=%.4f\n", k1, k2, k3);
            Serial.printf("  K4=%.4f  K5=%.4f  Trim=%.3f deg\n", k4, k5, pitchTrim);
            Serial.printf("  Motors: %s  State: %s\n",
                          steppers.isEnabled() ? "ON" : "OFF",
                          state == STATE_BALANCING ? "BALANCING" : "IDLE");
            Serial.println("----------------------------------------\n");
            break;
        case '?':
            Serial.println("\nE enable | X stop | C cal gyro | S settings | L debug | R reset");
            Serial.println("K1=..K5= LQR gains   T= trim (deg)   M= motor current (mA)");
            Serial.println("Radxa: V,<fwd -1..1>,<steer -1..1>,<en 0|1>\n");
            break;
        default: break;
    }
}

void pollSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxLen > 0) { rxBuf[rxLen] = '\0'; handleLine(rxBuf); rxLen = 0; }
        } else if (rxLen < sizeof(rxBuf) - 1) {
            rxBuf[rxLen++] = c;
        } else {
            rxLen = 0;   // overflow: drop garbage line
        }
    }
}

// ============================================================
//  CONTROLLER — LQR / LQI full-state feedback
// ============================================================
void runController(float dt) {

    float th = pitchF * DEG_TO_RAD;      // rad
    float om = rateF  * DEG_TO_RAD;      // rad/s

    float vIn = cmdFwd * MAX_DRIVE_VEL_MS;
    bool driving = fabsf(vIn) > (DRIVE_DEADBAND * MAX_DRIVE_VEL_MS);

    if (driving) {
        // Velocity-tracking mode: K2..K4 subset (poles verified stable).
        // Hold point is dragged along with a braking lookahead so the
        // robot parks smoothly where it stops.
        xRef = posM + velF * BRAKE_LOOKAHEAD_S;
        zInt = 0.0f;
        uOut = -(k2 * (velF - vIn) + k3 * th + k4 * om);
    } else {
        // Position hold / go-to: full LQI vector.
        float ex = posM - xRef;
        zInt = clampf(zInt + ex * dt, -Z_INT_LIM, Z_INT_LIM);
        uOut = -(k1 * ex + k2 * velF + k3 * th + k4 * om + k5 * zInt);
    }

    uOut = clampf(uOut, -A_MAX_MS2, A_MAX_MS2);
    vCmd = clampf(vCmd + uOut * dt, -V_MAX_MS, V_MAX_MS);
    float baseSteps = vCmd * STEPS_PER_M;

    // ---- yaw: commanded turn or encoder-differential heading hold ----
    if (fabsf(cmdSteer) > STEER_DEADBAND) {
        float target = cmdSteer * MAX_STEER_STEPS;
        float slew = STEER_SLEW * dt;
        steerSteps += clampf(target - steerSteps, -slew, slew);
        diffTarget = encDiff;                       // re-latch heading
    } else {
        float dErr = diffTarget - encDiff;
        steerSteps = clampf(dErr * YAW_HOLD_KP - diffRateF * YAW_HOLD_KD,
                            -MAX_STEER_STEPS, MAX_STEER_STEPS);
    }

    steppers.setSpeeds((int32_t)(baseSteps - steerSteps),
                       (int32_t)(baseSteps + steerSteps));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(300);
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);

    Serial.println("\n==================================================");
    Serial.println("     2PyBot BaseLink — Package B: LQR / LQI");
    Serial.println("==================================================");

    if (!imu.begin()) {
        Serial.println("[MAIN] FATAL: IMU init failed — halting");
        while (true) delay(1000);
    }
    if (!steppers.begin()) {
        Serial.println("[MAIN] WARNING: TMC2208 UART unresponsive");
    }

    Serial.printf("[MAIN] Settling sensors %d ms — keep robot still\n", STARTUP_SETTLE_MS);
    delay(STARTUP_SETTLE_MS);
    imu.calibrateGyro();

    espnow_receiver_begin();

    Serial.printf("[MAIN] STEPS_PER_M=%.0f  COUNTS_PER_M=%.0f  Vmax=%.2f m/s  Amax=%.2f m/s^2\n",
                  STEPS_PER_M, COUNTS_PER_M, V_MAX_MS, A_MAX_MS2);
    Serial.println("[MAIN] Send 'E' (or Radxa V,..,1 / joystick) to arm. '?' for help.\n");

    digitalWrite(ONBOARD_LED, LOW);
    lastLoopUs = micros();
}

// ============================================================
//  MAIN LOOP — fixed 200 Hz
// ============================================================
void loop() {
    unsigned long nowUs = micros();
    unsigned long elapsed = nowUs - lastLoopUs;
    if (elapsed < LOOP_PERIOD_US) return;
    lastLoopUs = nowUs;
    float dt = elapsed / 1000000.0f;
    if (dt > 0.05f) dt = 0.05f;               // guard after stalls

    // ---- 1. sensors ----
    imu.update(dt);
    pitchF = PITCH_FWD_SIGN * (imu.getPitch() - pitchTrim);
    rateF  = RATE_FWD_SIGN  *  imu.getPitchRate();

    encRawL = steppers.getPositionL();
    encRawR = steppers.getPositionR();
    float posL = ENC_FWD_SIGN * (float)encRawL;
    float posR = ENC_FWD_SIGN * (float)encRawR;

    static float posPrev = 0.0f; static bool posInit = false;
    posM = 0.5f * (posL + posR) / COUNTS_PER_M;
    if (!posInit) { posPrev = posM; posInit = true; }
    float vRaw = (posM - posPrev) / dt;
    posPrev = posM;
    const float aV = 1.0f - expf(-2.0f * PI * 15.0f / LOOP_FREQ_HZ);  // 15 Hz EMA
    velF += aV * (vRaw - velF);

    static float diffPrev = 0.0f; static bool diffInit = false;
    encDiff = posR - posL;
    if (!diffInit) { diffPrev = encDiff; diffInit = true; }
    float dRaw = (encDiff - diffPrev) / dt;
    diffPrev = encDiff;
    diffRateF += aV * (dRaw - diffRateF);

    // ---- 2. inputs ----
    pollSerial();

    // ESP-NOW enable edge (works even when Radxa has drive authority)
    uint8_t jEn = joyEnable;
    if (jEn && !joyEnPrev)  requestEnable();
    if (!jEn && joyEnPrev)  requestDisable("joystick disable");
    joyEnPrev = jEn;

    // Drive-authority arbitration: Radxa fresh > joystick fresh > zero
    bool radxaFresh = (millis() - radxaLastRx) < RADXA_TIMEOUT_MS && radxaLastRx != 0;
    bool joyFresh   = (millis() - lastJoyPacketMs) < JOY_TIMEOUT_MS_CFG && lastJoyPacketMs != 0;
    if (radxaFresh) {
        cmdFwd = radxaFwd;  cmdSteer = radxaSteer;
    } else if (joyFresh) {
        cmdFwd   = clampf(joyForward * JOY_FWD_SCALE,  -1.0f, 1.0f);
        cmdSteer = clampf(joySteering * JOY_STEER_SCALE, -1.0f, 1.0f);
    } else {
        cmdFwd = 0.0f; cmdSteer = 0.0f;
    }

    // ---- 3. state machine ----
    if (state == STATE_IDLE) {
        if (motorsRequested && fabsf(pitchF) < ARM_ANGLE_DEG) {
            resetController();
            steppers.enable();
            state = STATE_BALANCING;
            Serial.println("[MAIN] -> BALANCING");
        }
        if (millis() - lastBlinkMs > 500) {
            lastBlinkMs = millis(); ledState = !ledState;
            digitalWrite(ONBOARD_LED, ledState);
        }
    } else { // BALANCING
        if (fabsf(pitchF) > MAX_TILT_ANGLE || fabsf(rateF) > MAX_PITCH_RATE_SAFETY) {
            requestDisable(fabsf(pitchF) > MAX_TILT_ANGLE ? "FELL" : "RATE SPIKE");
        } else {
            runController(dt);
            digitalWrite(ONBOARD_LED, HIGH);
        }
    }

    // ---- 4. odometry stream to Radxa, 50 Hz, always on ----
    uint32_t nowMs = millis();
    if (nowMs - lastOdomMs >= ODOM_PERIOD_MS && RADXA_STREAM) {
        lastOdomMs = nowMs;
        Serial.printf("O,%lu,%lld,%lld,%.2f,%.2f\n",
                      (unsigned long)nowMs,
                      (long long)encRawL, (long long)encRawR,
                      imu.getPitch(), imu.getYaw());
    }

    // ---- 5. optional human-readable debug, 10 Hz ----
    if (DEBUG_STREAM && nowMs - lastDbgMs >= DEBUG_PERIOD_MS) {
        lastDbgMs = nowMs;
        Serial.printf("D pF=%+6.2f w=%+7.1f u=%+5.2f v=%+5.2f ex=%+6.3f vCmd=%+6.0f st=%s\n",
                      pitchF, rateF, uOut, velF, posM - xRef,
                      vCmd * STEPS_PER_M,
                      state == STATE_BALANCING ? "BAL" : "IDLE");
    }
}
