/*
 * ============================================================
 *  BaseLink.ino — Self-Balancing Robot Firmware
 * ============================================================
 *  Phase 1: Core self-balancing and locomotion control.
 *
 *  Hardware:
 *    - ESP32 DevKit V1
 *    - Dual NEMA17 steppers driven by TMC2208
 *    - QMC5883L magnetometer
 *    - ISM6HG256X IMU (I2C)
 *    - MT6816 magnetic encoders (PCNT quadrature)
 *
 *  Serial commands (USB or Bluetooth):
 *    P=15  I=0  D=0.8  T=0  — Adjust PID gains and setpoint live.
 *    E / X                  — Enable / emergency-stop motors.
 *    S                      — Print current parameter state.
 *    ?                      — Print command reference.
 *
 *  Author: Atharva
 *  Date:   May 2026
 * ============================================================
 */

#include "config.h"
#include "pid_controller.h"
#include "imu_sensor.h"
#include "stepper_control.h"
#include "serial_tuner.h"
#include "espnow_comm.h"
#include <BluetoothSerial.h>

IMUSensor imu;

// Primary balance PID.
PIDController pid(
    DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
    DEFAULT_TARGET_ANGLE,
    PID_OUTPUT_MIN, PID_OUTPUT_MAX,
    INTEGRAL_LIMIT,
    true, ADAPTIVE_ERROR_THRESHOLD, ADAPTIVE_KP_BOOST,
    PID_D_FILTER_ALPHA
);

// Heading-hold (yaw) PID.
PIDController yawPid(
    YAW_KP, YAW_KI, YAW_KD,
    0.0f,
    -MAX_STEERING, MAX_STEERING,
    1000.0f
);

BluetoothSerial SerialBT;

// Drive setpoints.
float targetAngleOffset = 0.0f;
float targetYaw = 0.0f;
float steeringOffset = 0.0f;
float targetSteering = 0.0f;

// Drive state.
char driveState = 'X';
int32_t latestLeftSpeed = 0;
int32_t latestRightSpeed = 0;

// Feature toggles.
bool invertYaw = true;
bool enableDriftCorrection = true;
bool enableYawPID = false;

// Manual drive parameters.
float dynamicTargetAngle = DEFAULT_TARGET_ANGLE;
float manualDriveTilt = 3.0f;

bool DEBUG = true;
float maxPosHoldTilt = MAX_POS_HOLD_TILT;

// ============================================================
//  DRIFT CORRECTION
// ============================================================
// Evaluates encoder displacement over a fixed 0.5 s window.
// A net tick delta exceeding DRIFT_ERROR_SETPOINT triggers a
// corrective lean opposite to the direction of drift.
#define DRIFT_EVAL_PERIOD_HZ 2.0f
#define DRIFT_EVAL_LOOPS    (int)(LOOP_FREQ_HZ / DRIFT_EVAL_PERIOD_HZ)
#define DRIFT_ERROR_SETPOINT 500

int64_t lastDriftEvalTicks = 0;
uint32_t driftLoopCounter = 0;
float activeDriftCorrection = 0.0f;

SerialTuner tuner(pid, steppers, imu);

// ============================================================
//  TIMING
// ============================================================
unsigned long lastLoopUs   = 0;
unsigned long loopCounter  = 0;
unsigned long lastPrintMs  = 0;
unsigned long lastBlinkMs  = 0;
bool onboardLedState       = false;

// ============================================================
//  STATE MACHINE
// ============================================================
enum RobotState {
    STATE_INIT,       // Hardware init and zeroing.
    STATE_IDLE,       // Motors disabled, awaiting command.
    STATE_BALANCING,  // Active closed-loop balance control.
    STATE_FALLEN      // Fall detected; motors cut. Awaits recovery.
};

RobotState state = STATE_INIT;

// ============================================================
//  SETUP
// ============================================================
void setup() {

    Serial.begin(SERIAL_BAUD);
    delay(500);

    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("                SELF-BALANCING ROBOT              ");
    Serial.println("==================================================");
    Serial.println();

    if (!imu.begin()) {
        Serial.println("[MAIN] FATAL: IMU init failed — halting");
        while (true) delay(1000);
    }

    if (!steppers.begin()) {
        Serial.println("[MAIN] WARNING: TMC2208 UART unresponsive — stepping will work, UART config will not");
    }

    Serial.printf("[MAIN] Settling sensors for %d ms\n", STARTUP_SETTLE_MS);
    delay(STARTUP_SETTLE_MS);

    if (!imu.calibrateGyro()) {
        Serial.println("[MAIN] WARNING: Gyro calibration reported insufficient samples");
    }

    digitalWrite(ONBOARD_LED, LOW);

    espnow_receiver_begin();
    SerialBT.begin("Self_Balancing_Robot");
    tuner.begin();

    state = STATE_IDLE;
    lastLoopUs = micros();

    Serial.println("\n[MAIN] Press 'E' to enable motors");
    Serial.println("[MAIN] Press '?' for command reference\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

    // Rate limiter — enforces fixed LOOP_FREQ_HZ execution rate.
    unsigned long nowUs = micros();
    unsigned long elapsedUs = nowUs - lastLoopUs;
    if (elapsedUs < LOOP_PERIOD_US) return;

    lastLoopUs = nowUs;
    float dt = elapsedUs / 1000000.0f;
    loopCounter++;

    // IMU update must execute first each cycle.
    imu.update(dt);
    float angle = imu.getPitch();

    tuner.process();

    // Transition IDLE -> BALANCING when motors are enabled and upright.
    if (steppers.isEnabled() && state == STATE_IDLE) {
        if (fabsf(angle) < 5.0f) {
            state = STATE_BALANCING;
            pid.reset();
            yawPid.reset();
            targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
            lastDriftEvalTicks = steppers.getAveragePosition();
            driftLoopCounter = 0;
            activeDriftCorrection = 0.0f;
            Serial.println("[MAIN] State -> BALANCING");
        }
    }

    // Transition BALANCING -> IDLE when motors are disabled externally.
    if (!steppers.isEnabled() && state == STATE_BALANCING) {
        state = STATE_IDLE;
        Serial.println("[MAIN] State -> IDLE");
    }

    // ============================================================
    //  BLUETOOTH COMMAND PARSER
    // ============================================================
    while (SerialBT.available()) {
        char c = (char)SerialBT.read();
        static String btBuffer = "";

        if (c == '\n') {
            if (btBuffer.startsWith("$")) {
                String cmd = btBuffer.substring(1);
                int eqIdx = cmd.indexOf('=');

                if (eqIdx > 0) {
                    String key = cmd.substring(0, eqIdx);
                    float val = cmd.substring(eqIdx + 1).toFloat();

                    if      (key == "KP")    pid.Kp = val;
                    else if (key == "KI")    pid.Ki = val;
                    else if (key == "KD")    pid.Kd = val;
                    else if (key == "T")     dynamicTargetAngle = val;
                    else if (key == "MDT")   manualDriveTilt = val;
                    else if (key == "PMT")   maxPosHoldTilt = val;
                    else if (key == "YKP")   yawPid.Kp = val;
                    else if (key == "YKD")   yawPid.Kd = val;
                    else if (key == "YMT")   { yawPid.outputMax = val; yawPid.outputMin = -val; }
                    else if (key == "INV_Y") invertYaw = (val > 0.5f);
                    else if (key == "EN_P") {
                        enableDriftCorrection = (val > 0.5f);
                        if (!enableDriftCorrection) targetAngleOffset = 0.0f;
                    }
                    else if (key == "EN_Y") {
                        bool en = (val > 0.5f);
                        if (en && !enableYawPID) {
                            targetYaw = imu.getYaw();
                            yawPid.reset();
                        }
                        enableYawPID = en;
                    }
                } else {
                    // Single-keyword commands.
                    String key = cmd;
                    if      (key == "E") { pid.reset(); targetAngleOffset = 0.0f; targetYaw = imu.getYaw(); steppers.enable(); }
                    else if (key == "X") { steppers.setSpeed(0); steppers.disable(); state = STATE_IDLE; }
                    else if (key == "C") { steppers.setSpeed(0); steppers.disable(); state = STATE_IDLE; imu.calibrateGyro(); }
                    else if (key == "L") { DEBUG = !DEBUG; }
                }
            }
            btBuffer = "";
        } else if (c == '#') {
            // Single-character joystick button commands.
            char nextC;
            if (SerialBT.readBytes(&nextC, 1) == 1) {
                nextC = toupper(nextC);
                bool wasSteering = (steeringOffset != 0.0f);
                bool wasDriving = (driveState != 'X');

                if (nextC == 'W' || nextC == 'S' || nextC == 'X') driveState = nextC;

                if      (nextC == 'A')              targetSteering = -TURN_SPEED_DEG_SEC;
                else if (nextC == 'D')              targetSteering =  TURN_SPEED_DEG_SEC;
                else if (nextC == ' ' || nextC == 'X') targetSteering = 0.0f;
            }
        } else {
            btBuffer += c;
        }
    }

    // ============================================================
    //  JOYSTICK INPUT (ESP-NOW)
    // ============================================================
    // Falls back to zero if no packet is received within JOY_TIMEOUT_MS.
    bool joyActive = (millis() - lastJoyPacketMs < JOY_TIMEOUT_MS);
    bool driving = false;

    if (joyActive) {
        float joyFwd = joyForward;
        targetSteering = joySteering * TURN_SPEED_DEG_SEC;

        if (fabsf(joyFwd) > 0.05f) {
            float desired = -joyFwd;
            float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
            targetAngleOffset += (desired - targetAngleOffset) * alpha;
            if (fabsf(targetAngleOffset - desired) < 0.02f) targetAngleOffset = desired;
            driving = true;
            // Reset drift state while actively driving.
            lastDriftEvalTicks = steppers.getAveragePosition();
            driftLoopCounter = 0;
            activeDriftCorrection = 0.0f;
        }

        // Edge detection for enable/disable button.
        static bool lastJoyEn = false;
        bool joyEn = (joyEnable != 0);
        if (joyEn && !lastJoyEn) {
            pid.reset();
            targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
            lastDriftEvalTicks = steppers.getAveragePosition();
            driftLoopCounter = 0;
            activeDriftCorrection = 0.0f;
            steppers.enable();
        } else if (!joyEn && lastJoyEn) {
            steppers.setSpeed(0);
            steppers.disable();
            state = STATE_IDLE;
        }
        lastJoyEn = joyEn;

    } else {
        // Keyboard W/S drive fallback when joystick is inactive.
        if (driveState == 'W') {
            targetAngleOffset -= MANUAL_DRIVE_RATE * dt;
            driving = true;
            lastDriftEvalTicks = steppers.getAveragePosition();
            driftLoopCounter = 0;
            activeDriftCorrection = 0.0f;
        } else if (driveState == 'S') {
            targetAngleOffset += MANUAL_DRIVE_RATE * dt;
            driving = true;
            lastDriftEvalTicks = steppers.getAveragePosition();
            driftLoopCounter = 0;
            activeDriftCorrection = 0.0f;
        }
    }

    // ============================================================
    //  IDLE DRIFT CORRECTION
    // ============================================================
    if (!driving) {
        float desired = 0.0f;

        if (enableDriftCorrection && state == STATE_BALANCING) {
            driftLoopCounter++;
            int64_t currentPos = steppers.getAveragePosition();

            if (driftLoopCounter >= DRIFT_EVAL_LOOPS) {
                int64_t netDelta = currentPos - lastDriftEvalTicks;

                if (abs((int32_t)netDelta) > DRIFT_ERROR_SETPOINT) {
                    // Lean opposite to the direction of drift.
                    activeDriftCorrection = (netDelta > 0) ? -maxPosHoldTilt : maxPosHoldTilt;
                } else {
                    activeDriftCorrection = 0.0f;
                }

                lastDriftEvalTicks = currentPos;
                driftLoopCounter = 0;
            }
            desired = activeDriftCorrection;
        } else {
            activeDriftCorrection = 0.0f;
        }

        // Exponential decay toward the drift correction target.
        float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
        targetAngleOffset += (desired - targetAngleOffset) * alpha;
        if (fabsf(targetAngleOffset - desired) < 0.02f) targetAngleOffset = desired;
    }

    targetAngleOffset = constrain(targetAngleOffset, -MAX_MANUAL_TILT, MAX_MANUAL_TILT);

    // ============================================================
    //  STEERING SMOOTHING
    // ============================================================
    float steerAlpha = 1.0f - expf(-STEER_SMOOTHING * dt);
    steeringOffset += (targetSteering - steeringOffset) * steerAlpha;
    if (fabsf(steeringOffset - targetSteering) < 1.0f) steeringOffset = targetSteering;

    if (fabsf(steeringOffset) > 0.1f) {
        targetYaw += steeringOffset * dt;
        while (targetYaw >  180.0f) targetYaw -= 360.0f;
        while (targetYaw < -180.0f) targetYaw += 360.0f;
    }

    // ============================================================
    //  STATE MACHINE
    // ============================================================
    switch (state) {

        case STATE_IDLE:
            // Slow LED blink in idle.
            if (millis() - lastBlinkMs >= 500) {
                lastBlinkMs = millis();
                onboardLedState = !onboardLedState;
                digitalWrite(ONBOARD_LED, onboardLedState ? HIGH : LOW);
            }
            break;

        case STATE_BALANCING: {

            // Fall detection — cut motors immediately.
            if (fabsf(angle) > MAX_TILT_ANGLE) {
                steppers.setSpeed(0);
                steppers.disable();
                state = STATE_FALLEN;
                Serial.printf("[MAIN] Fell at angle=%.1f° — control disabled\n", angle);
                break;
            }

            // LED blink rate scales with tilt angle.
            float absAngle = fabsf(angle);
            long blinkInterval = map(constrain((long)absAngle, 0, 15), 0, 15, 1000, 40);
            if (millis() - lastBlinkMs >= blinkInterval) {
                lastBlinkMs = millis();
                onboardLedState = !onboardLedState;
                digitalWrite(ONBOARD_LED, onboardLedState ? HIGH : LOW);
            }

            // 1. Compose balance setpoint.
            pid.setpoint = dynamicTargetAngle + targetAngleOffset;

            // 2. Compute balance PID output.
            float targetOutput = pid.compute(angle, dt);

            // 3. Compute yaw correction.
            float yawOutput = 0.0f;
            if (enableYawPID) {
                yawPid.setpoint = targetYaw;
                yawOutput = yawPid.computeAngle(imu.getYaw(), dt);
                if (invertYaw) yawOutput = -yawOutput;
            } else {
                // Manual proportional yaw mapping; keep ghost target aligned.
                yawOutput = steeringOffset * (MAX_STEERING / TURN_SPEED_DEG_SEC);
                targetYaw = imu.getYaw();
            }

            // 4. Rate-limit speed changes to prevent stepper stall.
            static float smoothedOutput = 0.0f;
            float maxChange = MOTOR_ACCEL_LIMIT * dt;
            float diff = targetOutput - smoothedOutput;
            if      (diff >  maxChange) smoothedOutput += maxChange;
            else if (diff < -maxChange) smoothedOutput -= maxChange;
            else                        smoothedOutput = targetOutput;

            // 5. Apply differential speeds for steering.
            latestLeftSpeed  = (int32_t)(smoothedOutput - yawOutput);
            latestRightSpeed = (int32_t)(smoothedOutput + yawOutput);
            steppers.setSpeeds(latestLeftSpeed, latestRightSpeed);

            break;
        }

        case STATE_FALLEN:
            digitalWrite(ONBOARD_LED, LOW);
            // Resume balancing once motors are re-enabled and angle is safe.
            if (steppers.isEnabled()) {
                if (fabsf(angle) < MAX_TILT_ANGLE) {
                    pid.reset();
                    state = STATE_BALANCING;
                    Serial.println("[MAIN] Recovered -> BALANCING");
                } else {
                    steppers.disable();
                    Serial.printf("[MAIN] Still fallen (%.1f°) — clear obstruction and re-enable\n", angle);
                }
            }
            break;

        default:
            break;
    }

    // ============================================================
    //  SERIAL TELEMETRY (5 Hz)
    // ============================================================
    if (loopCounter % (LOOP_FREQ_HZ / 5) == 0 && DEBUG) {
        Serial.printf("Angle: %5.1f | SP: %5.2f | P: %6.1f | I: %6.1f | D: %6.1f | DriftCorr: %5.2f | LeftSpd: %5d\n",
                      angle, pid.setpoint, pid.getP(), pid.getI(), pid.getD(), activeDriftCorrection, latestLeftSpeed);
    }
}
