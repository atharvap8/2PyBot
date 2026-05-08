/*
 * ============================================================
 *  Self_Balancing_Robot.ino
 * ============================================================
 *  Phase 1 — Core Self-Balancing and Locomotion Physics
 *
 *  Hardware Configuration:
 *    - ESP32 Development Board (38-pin definition).
 *    - Dual NEMA17 Stepper Motors driven securely by TMC2208 
 *      hardware operating via discrete UART manipulation.
 *    - ISM6HG256X high-precision IMU interacting over I2C.
 *
 *  Control Loop Execution (Precisely 200 Hz):
 *    1. Interrogates IMU strictly → fuses raw vectors via Mahony AHRS 
 *       → extracts discrete pitch and yaw Euler metrics.
 *    2. Dual PID controllers actively assess algorithmic setpoints 
 *       → resolves reactive motor speed commands.
 *    3. Asynchronous hardware timer ISR generates discrete 
 *       step pulses flawlessly decoupled from the mainline 
 *       processing loop.
 *
 *  Available Serial Command Interfaces:
 *    P=15  I=0  D=0.8  T=0  — Modify PID configurations live without code reflashing.
 *    E / X                  — Assert enable / trigger emergency motor disconnect.
 *    S                      — Render persistent algorithmic settings.
 *    ?                      — Render command syntax help documentation.
 *
 *  Author: Atharva
 *  Date:   April 2026
 * ============================================================
 */

#include "config.h"
#include "pid_controller.h"
#include "imu_sensor.h"
#include "stepper_control.h"
#include "serial_tuner.h"
#include "espnow_comm.h"
#include <BluetoothSerial.h>

// ============================================================
//  GLOBAL ABSTRACTION OBJECTS
// ============================================================

// Instantiates the primary inertial mathematical fusion construct.
IMUSensor imu;

// Constructs the primary PID structure rigidly governing fundamental gravity balancing.
PIDController pid(
    DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
    DEFAULT_TARGET_ANGLE,
    PID_OUTPUT_MIN, PID_OUTPUT_MAX,
    INTEGRAL_LIMIT,
    true, ADAPTIVE_ERROR_THRESHOLD, ADAPTIVE_KP_BOOST,
    PID_D_FILTER_ALPHA
);

// Constructs a supplementary PID strictly managing rotational velocity (Heading Hold).
PIDController yawPid(
    YAW_KP, YAW_KI, YAW_KD,
    0.0f,
    -MAX_STEERING, MAX_STEERING,
    1000.0f
);

// Instantiates the core Bluetooth stack providing wireless serial telemetry output.
BluetoothSerial SerialBT;

// ============================================================
//  PHYSICS ENGINE STATE VARIABLES
// ============================================================

// Target modifiers driving physical translation relative to absolute setpoint.
float targetAngleOffset = 0.0f;
float targetYaw = 0.0f;
float steeringOffset = 0.0f;
float targetSteering = 0.0f;

// Operational control paradigms defining autonomous inputs.
char driveState = 'X'; // Logical driving state tracking.
int32_t latestLeftSpeed = 0;
int32_t latestRightSpeed = 0;

// Logical toggles modifying secondary calculus parameters dynamically.
bool invertYaw = true; // Flips fundamental PID evaluation parity.
bool enableDriftCorrection = true; // Actuates the algorithmic Odometry position hold constraint.
bool enableYawPID = false; // Toggles strictly between manual yaw mapping vs active PID control.

// Primary constants influencing manual user driving capability logic.
float dynamicTargetAngle = DEFAULT_TARGET_ANGLE;
float manualDriveTilt = 3.0f;

// Variable tracking structural displacement actively during position-hold procedures.
bool DEBUG = true; // Toggles high-frequency serial evaluation rendering via UART.
float maxPosHoldTilt = MAX_POS_HOLD_TILT;

// Windowed tick-rate drift detection.
// Normal balancing: ticks oscillate up/down, net change over window ≈ 0.
// Drifting: ticks grow monotonically in one direction, net change is large.
// We keep a circular buffer of encoder positions spanning DRIFT_WINDOW_SEC.
#define DRIFT_WINDOW_TICKS  60       // 60 samples at 200Hz = 0.3 seconds window.
#define DRIFT_TICK_THRESHOLD 300     // Net encoder ticks over window to consider drift.
#define DRIFT_GAIN          0.0008f  // Corrective lean (degrees) per net tick of drift.
int64_t driftBuffer[DRIFT_WINDOW_TICKS];  // Circular buffer of avg encoder positions.
uint8_t driftBufIdx = 0;                   // Current write index.
bool    driftBufFilled = false;            // True once buffer has wrapped once (full window of data).

// Instantiates the live parsing engine consuming Serial updates actively.
SerialTuner tuner(pid, steppers, imu);

// ============================================================
//  EXECUTION TIMING METRICS
// ============================================================
unsigned long lastLoopUs   = 0;
unsigned long loopCounter  = 0;
unsigned long lastPrintMs  = 0;
unsigned long lastBlinkMs  = 0;
bool onboardLedState       = false;

// ============================================================
//  STATE MACHINE DEFINITIONS
// ============================================================
// Represents the strict chronological state sequence prohibiting 
// unpredictable mechanical execution states.
enum RobotState {
    STATE_INIT,        // Hardware bootstrapping and mathematical zeroing.
    STATE_IDLE,        // Safe state, motors disabled physically, pending user input.
    STATE_BALANCING,   // Active physics evaluation running in closed loops.
    STATE_FALLEN       // Catastrophic error state disabling hardware following a violation.
};

RobotState state = STATE_INIT;

// ============================================================
//  HARDWARE BOOTSTRAPPING
// ============================================================
void setup() {
    
    // Establishing native USB diagnostic communications.
    Serial.begin(SERIAL_BAUD);
    delay(500);
    
    // Assert visual feedback indicator LED securely.
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("                SELF-BALANCING ROBOT              ");
    Serial.println("==================================================");
    Serial.println();

    // ---- IMU Instantiation ----
    if (!imu.begin()) {
        Serial.println("[MAIN] FATAL: IMU logic fault — halting execution safely.");
        while (true) delay(1000);
    }

    // ---- Hardware Stepper / ISR Initialization ----
    if (!steppers.begin()) {
        Serial.println("[MAIN] WARNING: Driver UART connection fundamentally severed.");
        Serial.println("[MAIN] Mechanical stepping functions normally but configurations fail.");
    }

    // ---- Mechanical Sensor Stabilization ----
    // Provides thermal and electrical settling intervals strictly crucial 
    // for achieving perfect mathematical zero-bias baselines.
    Serial.printf("[MAIN] Applying algorithmic %d ms block ensuring sensor stabilization...\n",
                  STARTUP_SETTLE_MS);
    delay(STARTUP_SETTLE_MS);

    // ---- Static Zero-Rate Gyro Calibration ----
    if (!imu.calibrateGyro()) {
        Serial.println("[MAIN] WARNING: Gyroscopic bias extraction structurally corrupted.");
    }

    digitalWrite(ONBOARD_LED, LOW);
    
    // ---- Instantiate ESP-NOW Network Stack ----
    espnow_receiver_begin();

    // ---- Execute Bluetooth Telemetry Output Interfaces ----
    SerialBT.begin("Self_Balancing_Robot");
    
    // Execute textual configuration parser sequence.
    tuner.begin();

    // ---- Safe Initialization Complete ----
    state = STATE_IDLE;
    lastLoopUs = micros();

    Serial.println("\n[MAIN] Evaluation Ready. Assert command 'E' asserting hardware enablement.");
    Serial.println("[MAIN] Command '?' reveals structural logic dictionaries.\n");
}

// ============================================================
//  PRIMARY EXECUTION THREAD
// ============================================================
void loop() {
    
    // ---- Chronological Frequency Enforcement ----
    unsigned long nowUs = micros();
    unsigned long elapsedUs = nowUs - lastLoopUs;

    // Explicitly prohibit loop execution if the precise periodic duration is not met.
    // Guarantees all dt (delta-time) mathematics are flawlessly consistent.
    if (elapsedUs < LOOP_PERIOD_US) return;

    lastLoopUs = nowUs;
    float dt = elapsedUs / 1000000.0f;   // Formulate duration identically in seconds.
    loopCounter++;

    // ---- Absolute Highest Priority: IMU Fusion Fetch ---- 
    // Reads register I2C data immediately reducing procedural latency variations.
    imu.update(dt);
    float angle = imu.getPitch();

    // ---- Poll Diagnostic Commands ----
    tuner.process();

    // ---- Autonomous State Intersections ----
    // Safely upgrades the core physics state immediately upon detecting active motors.
    if (steppers.isEnabled() && state == STATE_IDLE) {
        
        // Strict boundary checking prohibits motor activation while physically leaned.
        if (fabsf(angle) < 5.0f) { 
            state = STATE_BALANCING;
            
            // Hard-reset algorithmic accumulators immediately, preventing massive integral jumps.
            pid.reset();
            yawPid.reset();
            
            // Initialize physics setpoints directly utilizing current physical metrics.
            targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
            driftBufIdx = 0;
            driftBufFilled = false;
            
            Serial.println("[MAIN] State execution logically proceeding → BALANCING");
        }
    }
    
    // Safely downgrades the physics state upon external logic disconnecting motor drivers.
    if (!steppers.isEnabled() && state == STATE_BALANCING) {
        state = STATE_IDLE;
        Serial.println("[MAIN] State execution securely reverted → IDLE");
    }

    // ---- Comprehensive Serial Telemetry String Interpreter ----
    // Consumes remote modifications updating parameters live directly parsing textual strings 
    // forwarded via Bluetooth wireless.
    while (SerialBT.available()) {
        char c = (char)SerialBT.read();
        static String btBuffer = "";

        if (c == '\n') {
            if (btBuffer.startsWith("$")) {
                // Synthesize the fundamental configuration modification payload.
                String cmd = btBuffer.substring(1);
                int eqIdx = cmd.indexOf('=');
                
                if (eqIdx > 0) {
                    
                    // Assigning key float variables algorithmically based upon String identifiers.
                    String key = cmd.substring(0, eqIdx);
                    float val = cmd.substring(eqIdx + 1).toFloat();
                    
                    if (key == "KP") pid.Kp = val;
                    else if (key == "KI") pid.Ki = val;
                    else if (key == "KD") pid.Kd = val;
                    else if (key == "T") dynamicTargetAngle = val;
                    else if (key == "MDT") manualDriveTilt = val;
                    else if (key == "PMT") maxPosHoldTilt = val;
                    else if (key == "YKP") yawPid.Kp = val;
                    else if (key == "YKD") yawPid.Kd = val;
                    else if (key == "YMT") { yawPid.outputMax = val; yawPid.outputMin = -val; }
                    else if (key == "INV_Y") invertYaw = (val > 0.5f);
                    else if (key == "EN_P") {
                        enableDriftCorrection = (val > 0.5f);
                        if (!enableDriftCorrection) {
                            targetAngleOffset = 0.0f;
                        }
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
                    
                    // Non-Float command modifiers altering immediate robot hardware sequences.
                    String key = cmd;
                    if (key == "E") { pid.reset(); targetAngleOffset = 0.0f; targetYaw = imu.getYaw(); steppers.enable(); }
                    else if (key == "X") { steppers.setSpeed(0); steppers.disable(); state = STATE_IDLE; }
                    else if (key == "C") { steppers.setSpeed(0); steppers.disable(); state = STATE_IDLE; imu.calibrateGyro(); }
                    else if (key == "L") { DEBUG = !DEBUG; }
                }
            }
            btBuffer = "";
        } else if (c == '#') {
            
            // Evaluates single immediate-execution keyboard commands sequentially.
            char nextC;
            if (SerialBT.readBytes(&nextC, 1) == 1) {
                nextC = toupper(nextC);
                bool wasSteering = (steeringOffset != 0.0f);
                bool wasDriving = (driveState != 'X');
                
                // Track forward and reverse commands uniquely logic.
                if (nextC == 'W' || nextC == 'S' || nextC == 'X') {
                    driveState = nextC;
                }
                
                // Track lateral and orientational differential modification logics.
                if (nextC == 'A') targetSteering = -TURN_SPEED_DEG_SEC;
                else if (nextC == 'D') targetSteering = TURN_SPEED_DEG_SEC;
                else if (nextC == ' ' || nextC == 'X') targetSteering = 0.0f;

            }
        } else {
            btBuffer += c;
        }
    }

    // ============================================================
    //  JOYSICK COMMAND EXECUTION WITH PRIORITY
    // ============================================================
    // Employs a fixed timeout validation. If external packets cease, the logic safely zero-defaults.
    bool joyActive = (millis() - lastJoyPacketMs < JOY_TIMEOUT_MS);
    bool driving = false;

    if (joyActive) {
        
        // --- Joystick Processing Engine ---
        float joyFwd = joyForward;
        targetSteering = joySteering * TURN_SPEED_DEG_SEC;
        
        // Prevent meaningless structural adjustments responding strictly to minute sensor noise floors.
        if (fabsf(joyFwd) > 0.05f) {
            
            // Calculates fundamental target lean mapping strictly from received network transmission degrees.
            float desired = -joyFwd; 
            
            // Utilize rigorous exponential damping curves restricting instantaneous target modification spikes.
            float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
            targetAngleOffset += (desired - targetAngleOffset) * alpha;
            
            // Formulates snapping tolerances reducing terminal evaluation overheads locally.
            if (fabsf(targetAngleOffset - desired) < 0.02f) targetAngleOffset = desired;
            driving = true;
            driftBufIdx = 0; driftBufFilled = false;  // Reset drift buffer when actively driving.
        }

        // --- Active Button State Interpretation ---
        static bool lastJoyEn = false;
        bool joyEn = (joyEnable != 0);
        
        // Executes strictly positive and negative logical assertion edges independently.
        if (joyEn && !lastJoyEn) {
            pid.reset(); targetAngleOffset = 0.0f;
            targetYaw = imu.getYaw();
            driftBufIdx = 0; driftBufFilled = false;
            steppers.enable();
        } else if (!joyEn && lastJoyEn) {
            steppers.setSpeed(0); steppers.disable(); state = STATE_IDLE;
        }
        lastJoyEn = joyEn;
        
    } else {
        
        // --- Redundant Keyboard Override Sequence ---
        // Incremental accumulation mechanism simulating analog sticks physically using raw 'W' 'A' 'S' 'D' keyholds.
        if (driveState == 'W') {
            targetAngleOffset -= MANUAL_DRIVE_RATE * dt;
            driving = true;
            driftBufIdx = 0; driftBufFilled = false;
        } else if (driveState == 'S') {
            targetAngleOffset += MANUAL_DRIVE_RATE * dt;
            driving = true;
            driftBufIdx = 0; driftBufFilled = false;
        }
    }


    // ============================================================
    //  IDLE POSITION-HOLD BRAKING CORRECTIONS
    // ============================================================
    if (!driving) {
        
        // Define baseline fallback target uniquely centered to absolute zero.
        float desired = 0.0f;
        
        // Windowed tick-rate drift detection.
        // Every tick: record encoder position into circular buffer.
        // Compare current vs 0.3s-ago position. Large net delta = drift.
        // Normal balancing oscillates, so net delta ≈ 0 over 0.3s.
        if (enableDriftCorrection && state == STATE_BALANCING) {
            
            int64_t currentPos = steppers.getAveragePosition();
            
            // Push current position into circular buffer.
            driftBuffer[driftBufIdx] = currentPos;
            driftBufIdx++;
            if (driftBufIdx >= DRIFT_WINDOW_TICKS) {
                driftBufIdx = 0;
                driftBufFilled = true;
            }
            
            // Only evaluate once we have a full window of data.
            if (driftBufFilled) {
                // The oldest sample in the buffer is at driftBufIdx (just wrapped).
                int64_t oldestPos = driftBuffer[driftBufIdx];
                int64_t netDelta = currentPos - oldestPos;
                
                if (abs((int32_t)netDelta) > DRIFT_TICK_THRESHOLD) {
                    // Monotonic drift detected — lean opposite.
                    desired = -(float)netDelta * DRIFT_GAIN;
                    desired = constrain(desired, -maxPosHoldTilt, maxPosHoldTilt);
                }
            }
        }
        
        // Exponentially filter positional decay curves creating flawlessly simulated mechanical inertia.
        float alpha = 1.0f - expf(-BRAKE_DECAY_RATE * dt);
        targetAngleOffset += (desired - targetAngleOffset) * alpha;
        
        if (fabsf(targetAngleOffset - desired) < 0.02f) targetAngleOffset = desired;
    }
    
    // Conclude parameter manipulation explicitly ensuring values remain enclosed fundamentally.
    targetAngleOffset = constrain(targetAngleOffset, -MAX_MANUAL_TILT, MAX_MANUAL_TILT);

    // ============================================================
    //  SMOOTH LATERAL STEERING EXECUTION
    // ============================================================
    // Resolves lateral momentum impulses through cascaded EMA structural filtering.
    float steerAlpha = 1.0f - expf(-STEER_SMOOTHING * dt);
    steeringOffset += (targetSteering - steeringOffset) * steerAlpha;
    if (fabsf(steeringOffset - targetSteering) < 1.0f) steeringOffset = targetSteering;

    // Dispatches rotational commands iteratively against localized structural Yaw tracking logic.
    if (fabsf(steeringOffset) > 0.1f) {
        targetYaw += steeringOffset * dt;
        
        // Condense targets consistently maintaining standardized ±180 angular constraints.
        while (targetYaw > 180.0f) targetYaw -= 360.0f;
        while (targetYaw < -180.0f) targetYaw += 360.0f;
    }

    // ============================================================
    //  STATE MACHINE EXECUTION RESOLUTION
    // ============================================================
    switch (state) {

        case STATE_IDLE:
            // Non-blocking timer implementation pulsing LEDs designating operational readiness.
            if (millis() - lastBlinkMs >= 500) {
                lastBlinkMs = millis();
                onboardLedState = !onboardLedState;
                digitalWrite(ONBOARD_LED, onboardLedState ? HIGH : LOW);
            }
            break;

       case STATE_BALANCING: {
            
            // ---- Safety Override Bounds Checking ----
            // Assert immediate software termination routines natively overriding balancing physics 
            // the second a critical geometric failure parameter is recorded.
            if (fabsf(angle) > MAX_TILT_ANGLE) {
                steppers.setSpeed(0);
                steppers.disable();
                state = STATE_FALLEN;
                Serial.printf("[MAIN] *** CRITICAL FAILURE EXCEEDED *** angle=%.1f° — physics disabled\n", angle);
                break;
            }

            // ---- Dynamic Operational LED Blinking ----
            // Scales blink intervals proportionally reflecting identical mechanical strain efforts.
            float absAngle = fabsf(angle);
            long blinkInterval = map(constrain((long)absAngle, 0, 15), 0, 15, 1000, 40);

            if (millis() - lastBlinkMs >= blinkInterval) {
                lastBlinkMs = millis();
                onboardLedState = !onboardLedState;
                digitalWrite(ONBOARD_LED, onboardLedState ? HIGH : LOW);
            }

            // ============================================================
            //  CASCADED ALGORITHMIC LOGIC GENERATION
            // ============================================================
            
            // 1. Synthesise definitive setpoint integrating hardware constants and dynamic modifications.
            pid.setpoint = dynamicTargetAngle + targetAngleOffset;
            
            // 2. Synthesise primitive torque variables demanding rotational compensation. 
            float targetOutput = pid.compute(angle, dt);
            
            // 3. Formulate entirely distinct lateral adjustments using independent feedback loops.
            float yawOutput = 0.0f;
            if (enableYawPID) {
                yawPid.setpoint = targetYaw;
                yawOutput = yawPid.computeAngle(imu.getYaw(), dt);
                if (invertYaw) yawOutput = -yawOutput;
            } else {
                // Instantiates simple proportional manual fallbacks bypassing PID layers conditionally.
                yawOutput = steeringOffset * (MAX_STEERING / TURN_SPEED_DEG_SEC); 
                targetYaw = imu.getYaw(); // Constantly realign ghost targets preserving subsequent enables.
            }

            // 4. Rate Limitation Physics Implementations.
            // Drastically curtails completely catastrophic processor step discontinuities entirely 
            // starving inductive coils mechanically if unfiltered.
            static float smoothedOutput = 0.0f;
            float maxChange = MOTOR_ACCEL_LIMIT * dt;
            float diff = targetOutput - smoothedOutput;
            
            if (diff > maxChange) smoothedOutput += maxChange;
            else if (diff < -maxChange) smoothedOutput -= maxChange;
            else smoothedOutput = targetOutput;

            // 5. Commit finalized output calculations merging independent logical matrices completely 
            // driving internal stepper registries globally.
            latestLeftSpeed  = (int32_t)(smoothedOutput - yawOutput);
            latestRightSpeed = (int32_t)(smoothedOutput + yawOutput);
            steppers.setSpeeds(latestLeftSpeed, latestRightSpeed);
            
            break;  
        }

        case STATE_FALLEN:
            digitalWrite(ONBOARD_LED, LOW);
            // Mandates manual intervention natively correcting absolute physical failure states 
            // explicitly before re-initiating closed loops.
            if (steppers.isEnabled()) {
                if (fabsf(angle) < MAX_TILT_ANGLE) {
                    pid.reset();
                    state = STATE_BALANCING;
                    Serial.println("[MAIN] Process resumed naturally → BALANCING");
                } else {
                    steppers.disable();
                    Serial.printf("[MAIN] Hardware fundamentally obstructed structurally (%.1f°) — require clearance\n", angle);
                }
            }
            break;

        default:
            break;
    }

    // ============================================================
    //  SYNCHRONIZED DIAGNOSTIC OUTPUT
    // ============================================================
    // Serializes extensive variable combinations utilizing structured string manipulations 
    // selectively routing datasets dynamically rendering external graphical interfaces effortlessly.
    if (loopCounter % PLOT_DIVIDER == 0 && DEBUG) {
        // Compute wheel angles from encoder ticks (0-360° within current revolution)
        int64_t encTicksL = steppers.getPositionL();
        int64_t encTicksR = steppers.getPositionR();
        int32_t modL = (int32_t)(encTicksL % ENCODER_CPR);
        int32_t modR = (int32_t)(encTicksR % ENCODER_CPR);
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
                      (long)encTicksL, (long)encTicksR,
                      wheelAngleL, wheelAngleR);
        Serial.print(buff);
        SerialBT.print(buff);
    }
}
