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
#include "fall_protection.h"
#include <BluetoothSerial.h>
#include <FastLED.h>

CRGB leds[LED_RING_COUNT];

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

// Predictive fall protection (soft input constraints).
FallProtection fallGuard;

// ============================================================
//  DRIFT CORRECTION
// ============================================================
float activeDriftCorrection = 0.0f;

// Positional PID (Drift Correction) parameters
float posKp = 0.0006f;
float posKd = 0.003f;
int64_t targetPos = 0;
float lastPosError = 0.0f;
bool wasDriving = false;

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
//  LED RING FEEDBACK
// ============================================================
void showLoadingAnimation() {
    static uint8_t pos = 0;
    static unsigned long lastUpdate = 0;
    static unsigned long animStart = 0;
    
    if (animStart == 0) animStart = millis();
    if (millis() - lastUpdate < 40) return;
    lastUpdate = millis();

    FastLED.clear();
    
    // Relative progress through the settle period (0.0 to 1.0)
    float progress = (float)(millis() - animStart) / STARTUP_SETTLE_MS;
    progress = constrain(progress, 0.0f, 1.0f);
    
    // Simple Red to Green transition (no Blue/Violet)
    CRGB color = CRGB(255 * (1.0f - progress), 255 * progress, 0);

    leds[pos] = color;
    // Simple trail
    leds[(pos + LED_RING_COUNT - 1) % LED_RING_COUNT] = color;
    leds[(pos + LED_RING_COUNT - 1) % LED_RING_COUNT].nscale8(128);

    FastLED.show();
    pos = (pos + 1) % LED_RING_COUNT;
}

void updateLEDMotionFeedback(float speedL, float speedR) {
    static float smoothedAvg = 0.0f;
    static float smoothedDiff = 0.0f;
    
    // Smoothing factor (0.1 = slow/smooth, 1.0 = instant/jittery)
    const float alpha = 0.15f; 
    
    float avgSpeed = (speedL + speedR) / 2.0f;
    float diff = speedR - speedL;
    
    // Apply Exponential Moving Average to eliminate jitter
    smoothedAvg += (avgSpeed - smoothedAvg) * alpha;
    smoothedDiff += (diff - smoothedDiff) * alpha;

    FastLED.clear();
    
    // Intensity mapping based on smoothed speed, starting from the deadband threshold.
    uint8_t intensity = (uint8_t)constrain(map(fabsf(smoothedAvg) + fabsf(smoothedDiff), LED_MOTION_DEADBAND, LED_MOTION_SPEED_MAX, 0, 255), 0, 255);
    CRGB color = CRGB(intensity, 255 - intensity, 0); // Green -> Red

    // Directional segments
    if (smoothedAvg > LED_MOTION_DEADBAND) {
        leds[14] = leds[15] = leds[0] = leds[1] = color;
    } else if (smoothedAvg < -LED_MOTION_DEADBAND) {
        leds[6] = leds[7] = leds[8] = leds[9] = color;
    }

    if (smoothedDiff > LED_MOTION_DEADBAND) {
        leds[2] = leds[3] = leds[4] = leds[5] = color;
    } else if (smoothedDiff < -LED_MOTION_DEADBAND) {
        leds[10] = leds[11] = leds[12] = leds[13] = color;
    }

    // Steady Green if effectively stopped (within deadband).
    if (fabsf(smoothedAvg) < LED_MOTION_DEADBAND && fabsf(smoothedDiff) < LED_MOTION_DEADBAND) {
        fill_solid(leds, LED_RING_COUNT, CRGB::Green);
    }

    FastLED.show();
}

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
    
    // LED Ring Initialization (Using WS2812B with GRB order)
    FastLED.addLeds<WS2812B, LED_RING_PIN, GRB>(leds, LED_RING_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);

    // Play loading animation during the sensor settle period.
    unsigned long startSettle = millis();
    while (millis() - startSettle < STARTUP_SETTLE_MS) {
        showLoadingAnimation();
        delay(20);
    }

    if (!imu.calibrateGyro()) {
        Serial.println("[MAIN] WARNING: Gyro calibration reported insufficient samples");
    }

    // --- Init Complete Animation ---
    // Slow fill
    for (int i = 0; i < LED_RING_COUNT; i++) {
        leds[i] = CRGB::Green;
        FastLED.show();
        delay(30); 
    }
    // Sharp flash
    fill_solid(leds, LED_RING_COUNT, CRGB::Black);
    FastLED.show();
    delay(150);
    fill_solid(leds, LED_RING_COUNT, CRGB::Green);
    FastLED.show();

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

    // Predictive fall-risk estimation (pitch relative to equilibrium).
    fallGuard.update(angle - dynamicTargetAngle, imu.getPitchRate(), dt);

    tuner.process();

    // 0. Update Encoders & Calculate Velocities
    static int64_t lastPosL = 0;
    static int64_t lastPosR = 0;
    int64_t currPosL = steppers.getPositionL();
    int64_t currPosR = steppers.getPositionR();
    int32_t encVelL = (int32_t)(currPosL - lastPosL);
    int32_t encVelR = (int32_t)(currPosR - lastPosR);
    lastPosL = currPosL;
    lastPosR = currPosR;

    // Transition IDLE -> BALANCING when motors are enabled and upright.
    if (steppers.isEnabled() && state == STATE_IDLE) {
        if (fabsf(angle) < 5.0f) {
            state = STATE_BALANCING;
            pid.reset();
            yawPid.reset();
            targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
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
                    else if (key == "PKP")   posKp = val;
                    else if (key == "PKD")   posKd = val;
                    else if (key == "YKP")   yawPid.Kp = val;
                    else if (key == "YKD")   yawPid.Kd = val;
                    else if (key == "YMT")   { yawPid.outputMax = val; yawPid.outputMin = -val; }
                    else if (key == "INV_Y") invertYaw = (val > 0.5f);
                    else if (key == "FP_EN") fallGuard.enabled = (val > 0.5f);
                    else if (key == "FP_H")  fallGuard.predictHorizonS = val;
                    else if (key == "FP_S")  fallGuard.riskStartDeg = val;
                    else if (key == "FP_F")  fallGuard.riskFullDeg = val;
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
        targetSteering = fallGuard.constrainSteering(joySteering * TURN_SPEED_DEG_SEC);

        if (fabsf(joyFwd) > 0.05f) {
            // Fall guard: cancel lean commands aimed toward a predicted
            // fall; de-rate all others while risk is elevated.
            float desired = fallGuard.constrainDrive(-joyFwd);
            float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
            targetAngleOffset += (desired - targetAngleOffset) * alpha;
            if (fabsf(targetAngleOffset - desired) < 0.02f) targetAngleOffset = desired;
            driving = true;
            activeDriftCorrection = 0.0f;
        }

        // Edge detection for enable/disable button.
        static bool lastJoyEn = false;
        bool joyEn = (joyEnable != 0);
        if (joyEn && !lastJoyEn) {
            pid.reset();
            targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
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
            activeDriftCorrection = 0.0f;
        } else if (driveState == 'S') {
            targetAngleOffset += MANUAL_DRIVE_RATE * dt;
            driving = true;
            activeDriftCorrection = 0.0f;
        }
        // Fall guard for keyboard drive: clamp the accumulated lean.
        if (driving && fallGuard.isActive()) {
            targetAngleOffset = fallGuard.constrainDrive(targetAngleOffset);
        }
        targetSteering = fallGuard.constrainSteering(targetSteering);
    }

    // ============================================================
    //  POSITIONAL PD (DRIFT CORRECTION)
    // ============================================================
    if (!driving) {
        if (wasDriving) {
            // Transition from driving to stationary: lock current position.
            targetPos = steppers.getAveragePosition();
            lastPosError = 0.0f;
            wasDriving = false;
        }

        float desired = 0.0f;
        // Handover Logic: Only allow positional corrections when the balance loop is in a calm, stabilized state.
        // This prevents the positional hold from "fighting" the PID during recovery.
        float pitchError = fabsf(angle - (dynamicTargetAngle + targetAngleOffset));
        bool stabilized = (pitchError < PITCH_ERROR_DEGREES); // Degrees

        if (enableDriftCorrection && state == STATE_BALANCING && stabilized) {
            int64_t currentPos = steppers.getAveragePosition();
            float error = (float)(targetPos - currentPos);
            float dError = (error - lastPosError) / dt;
            lastPosError = error;

            // Deadband to ignore micro-oscillations and sensor noise.
            if (fabsf(error) > 15.0f) {
                activeDriftCorrection = (error * posKp) + (dError * posKd);
                activeDriftCorrection = constrain(activeDriftCorrection, -maxPosHoldTilt, maxPosHoldTilt);
                desired = activeDriftCorrection;
            } else {
                desired = activeDriftCorrection; // Maintain current correction
            }
        } else {
            // If unstable or disabled, decay targetAngleOffset toward 0 to let the balance PID take full priority.
            desired = 0.0f;
        }

        // Smooth transition between balance-only and position-hold modes.
        float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
        targetAngleOffset += (desired - targetAngleOffset) * alpha;
        if (fabsf(targetAngleOffset - desired) < 0.01f) targetAngleOffset = desired;
    } else {
        wasDriving = true; // Bot is actively being driven; don't fight it.
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
            // Steady Green in IDLE means "Ready".
            fill_solid(leds, LED_RING_COUNT, CRGB::Green);
            FastLED.show();
            
            if (millis() - lastBlinkMs >= 500) {
                lastBlinkMs = millis();
                onboardLedState = !onboardLedState;
                digitalWrite(ONBOARD_LED, onboardLedState ? HIGH : LOW);
            }
            break;

        case STATE_BALANCING: {

            // Fall/Spike detection — cut motors immediately.
            if (fabsf(angle) > MAX_TILT_ANGLE || fabsf(imu.getPitchRate()) > MAX_PITCH_RATE_SAFETY) {
                steppers.setSpeed(0);
                steppers.disable();
                state = STATE_FALLEN;
                if (fabsf(angle) > MAX_TILT_ANGLE) {
                    Serial.printf("[MAIN] Fell at angle=%.1f° — control disabled\n", angle);
                } else {
                    Serial.printf("[MAIN] SPIKE DETECTED (%.1f°/s) — control disabled\n", imu.getPitchRate());
                }
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

            // Differential Damping: Prevent pivoting by opposing uneven wheel velocities.
            if (!driving && state == STATE_BALANCING) {
                float diffVelocity = (float)(encVelL - encVelR);
                yawOutput += diffVelocity * 0.5f; 
            }

            // Fall guard: constrain the differential split while at risk
            // so mid-turn wheel authority returns to the balance loop.
            yawOutput = fallGuard.constrainYawOutput(yawOutput);

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
            
            // 6. Visual feedback based on speed and direction.
            if (fallGuard.isActive()) {
                // Amber pulse signals constrained inputs / fall risk.
                uint8_t pulse = (uint8_t)(128 + 127 * sinf(millis() * 0.02f));
                fill_solid(leds, LED_RING_COUNT, CRGB(pulse, (uint8_t)(pulse * 0.55f), 0));
                FastLED.show();
            } else {
                updateLEDMotionFeedback((float)latestLeftSpeed, (float)latestRightSpeed);
            }

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
    if (loopCounter % PLOT_DIVIDER == 0 && DEBUG) {
        // Compute wheel angles from encoder ticks
        int32_t modL = (int32_t)(currPosL % ENCODER_CPR);
        int32_t modR = (int32_t)(currPosR % ENCODER_CPR);
        if (modL < 0) modL += ENCODER_CPR;
        if (modR < 0) modR += ENCODER_CPR;
        float wheelAngleL = (float)modL / ENCODER_CPR * 360.0f;
        float wheelAngleR = (float)modR / ENCODER_CPR * 360.0f;
        
        // 20 tab-separated fields: original 16 + 4 encoder fields
        char buff[320];
        snprintf(buff, sizeof(buff), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.1f\t%.1f\t%.1f\t%d\t%d\t%ld\t%ld\t%.1f\t%.1f\n",
                      angle,
                      pid.setpoint,
                      imu.getAx(), imu.getAy(), imu.getAz(),
                      imu.getGx(), imu.getGy(), imu.getGz(),
                      imu.getMx(), imu.getMy(), imu.getMz(),
                      pid.getP(), pid.getI(), pid.getD(),
                      latestLeftSpeed, latestRightSpeed,
                      (long)currPosL, (long)currPosR,
                      wheelAngleL, wheelAngleR);
        Serial.print(buff);
        SerialBT.print(buff);
    }
}
