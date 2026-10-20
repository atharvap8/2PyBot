/*
 * ============================================================
 *  BaseLink.ino — 2PyBot v4.0.2, M0601 BLDC hubs + LQR / LQI
 * ============================================================
 *  Control law: optimal full-state feedback on the linearized
 *  velocity-driven inverted pendulum, augmented with a position
 *  integral (LQI) that nulls drift from the unknown CoG offset.
 *      u = -( K1*(x-xRef) + K2*v + K3*pitch + K4*pitchRate + K5*z )
 *      z = Integral(x - xRef),   u = wheel acceleration
 *      vCmd += u*dt  ->  common/differential current loops -> RS485
 *
 *  Gains were computed offline by solving the continuous-time 
 *  algebraic Riccati equation (see scripts/lqr_design.py,
 *  which also regenerates them for different geometry). Validated
 *  in simulation: 0.5 cm RMS station keeping under a 0.35 deg CoG
 *  error, 2.3 deg peak excursion for a 45 deg/s shove, < 0.3 cm
 *  steady-state error on a 0.5 m position command.
 *
 *  Radxa serial bridge (USB serial, 460800):
 *    OUT 50 Hz :  O,<millis>,<encL>,<encR>,<pitchDeg>,<yawDeg>\n
 *    IN        :  V,<forward -1..1>,<steering -1..1>,<enable 0|1>\n
 *
 *  Tuner commands (same serial port, newline terminated):
 *    E enable    X e-stop    C cal gyro    S settings
 *    L toggle debug stream   R reset controller   ? help
 *    K1=..K5=<v> LQR gains   T=<v> pitch trim (deg)   M=<mA> current
 *
 *  EVOFOX One S pairs DIRECTLY over Bluetooth (Bluepad32 board
 *  core; ESP-NOW removed so BT owns the radio). The Radxa keeps
 *  override authority while its 'V' lines are fresh -- the
 *  vision guard works unchanged.
 * ============================================================
 */

#include "config.h"
#include "params.h"
#include "autotune.h"
#include "terrain.h"
#include "tof_guard.h"
#include "startup_gate.h"
#include "fall_prevention.h"
#include "auto_climb.h"
#include "descent_guard.h"
#include "navigation_lease.h"
#include "imu_sensor.h"
#include "hub_drive.h"
#include "bt_gamepad.h"    // EVOFOX One S DIRECT Bluetooth (replaces espnow_comm.h)
#include "led_ring.h"      // WS2812 16-LED expression ring on GPIO 15
#include "payload.h"       // servos GPIO 26/0 + torch MOSFET GPIO 2

IMUSensor imu;

// ============================================================
//  STATE
// ============================================================
enum RobotState { STATE_IDLE, STATE_BALANCING };
RobotState state       = STATE_IDLE;
bool motorsRequested   = false;
bool DEBUG_STREAM      = true;
bool RADXA_STREAM      = true;
bool hostBootReady     = false;
StartupGate startupGate;
bool hardwareReady = false;

// Forward-positive measurements (see config.h sign conventions)
float pitchF = 0.0f, rateF = 0.0f;      // deg, deg/s
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
hub_control::Steering yawControl;

// ---- telemetry instrumentation (read-only, no control effect) ----
uint8_t  satV = 0, satA = 0;     // clamp hit this cycle: velocity cap / accel limit
uint32_t loopMaxUs = 0;          // worst loop period since last D line

// ---- expression features (gamepad) ----
bool  stiffHold = false;    // "H,1": high-stiffness hold gain set
bool  climbMode = false;    // "H,2" / D-pad DOWN: slope LQI mode
float lookCmd   = 0.0f;     // "A,x": sustained yaw offset, -1..1

// Gesture engine: keyframes of (duration, fwd, steer) fed into the
// normal drive inputs. Balancer + safety limits stay in charge.
struct GStep { uint16_t ms; float fwd; float steer; };
static const GStep GEST_YES[]   = {{350,0.50f,0},{350,-0.50f,0},{350,0.50f,0},{350,-0.50f,0},{200,0,0}};
static const GStep GEST_NO[]    = {{250,0,0.50f},{250,0,-0.50f},{250,0,0.50f},{250,0,-0.50f},{250,0,0.50f},{250,0,-0.50f},{150,0,0}};
static const GStep GEST_SPIN[]  = {{2400,0,0.55f},{250,0,0}};
static const GStep GEST_DANCE[] = {{300,0.50f,0},{300,-0.50f,0},{500,0,0.60f},{500,0,-0.60f},
                                   {300,0.50f,0},{300,-0.50f,0},{1200,0,0.55f},{1200,0,-0.55f},
                                   {300,-0.50f,0},{300,0.50f,0},{250,0,0}};
const GStep* gestSeq = nullptr;
uint8_t  gestLen = 0, gestIdx = 0;
uint32_t gestStepStart = 0;

// Gains and trim live in the parameter store (params.h) so the GUI, the app
// and the legacy K1=/T= serial tuners all write the SAME memory. These
// aliases keep the controller code below reading exactly as it always did.
// (Without them the sliders write P.* while the loop reads a private copy --
//  the tuning appears to do nothing at all.)
#define k1        P.k1
#define k2        P.k2
#define k3        P.k3
#define k4        P.k4
#define k5        P.k5
#define pitchTrim P.pitchTrim

// Drive inputs after link arbitration
float cmdFwd = 0.0f, cmdSteer = 0.0f;

// Radxa link
uint32_t radxaLastRx = 0;
float    radxaFwd = 0.0f, radxaSteer = 0.0f;
uint8_t  radxaEn = 0, radxaEnPrev = 0;
ToFGuard tofGuard;
FallPrevention fallPrevention;
AutoClimb autoClimb;
DescentGuard descentGuard;
NavigationLease navigation;
float climbXRef=0.0f,climbIntegral=0.0f,climbDriveRef=0.0f,climbRecovery=0.0f;
bool climbReferenceInitialized=false;
float requestedDriveV = 0.0f;
float appliedBalanceScale = 1.0f;
float wheelTargetL = 0.0f, wheelTargetR = 0.0f;
float appliedSteerSteps = 0.0f;
bool driveReferenceActive = false;


// Serial line buffer
char  rxBuf[96];
uint8_t rxLen = 0;

// Timing
unsigned long lastLoopUs = 0;
uint32_t lastOdomMs = 0, lastDbgMs = 0, lastBlinkMs = 0;
bool ledState = false;

// ============================================================
//  Hooks the auto-tune routines need from the sketch
// ============================================================
bool at_isBalancing() { return state == STATE_BALANCING; }
void at_motorsOff()   { hub.disable(); }
bool params_motionActive() { return hub.isEnabled() || motorsRequested; }
void at_setWheelSpeed(float stepsPerSec) {
    if(startupGate.maintenance)return;
    if (!hub.isEnabled()) hub.enable();
    hub.setSpeeds(stepsPerSec, stepsPerSec);
}

// ============================================================
//  params_applyHardware() — called by params.h on every write.
//  Keeps hardware in step with the parameter store. Anything that
//  is read fresh each loop (gains, limits, deadbands) needs nothing
//  here; only latched hardware settings do.
// ============================================================
void params_applyHardware(bool currentChanged) {
    if (!hardwareReady) return;        // NVS/metadata remain usable before hardware begin()
    imu.setFilterCutoff(P.imuCutoff);
    (void)currentChanged;
    hub.applyTuning();
}

// ============================================================
//  HELPERS
// ============================================================
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void resetController() {
    navigation.reset();
    fallPrevention.reset();
    autoClimb.resetDynamics();
    descentGuard.reset();
    climbXRef=posM;climbIntegral=climbDriveRef=climbRecovery=0;
    climbReferenceInitialized=false;
    requestedDriveV=0.0f;appliedBalanceScale=1.0f;wheelTargetL=wheelTargetR=0.0f;
    appliedSteerSteps=0.0f;
    driveReferenceActive=false;
    vCmd = 0.0f; zInt = 0.0f; uOut = 0.0f;
    xRef = posM;
    steerSteps = 0.0f; diffTarget = encDiff;
    yawControl.reset(encDiff);
    gestSeq = nullptr;stiffHold=autoClimb.policy==AutoClimb::FORCE_STIFF;climbMode=false;lookCmd=0.0f;
}

void startGesture(const GStep* seq, uint8_t len, const char* name) {
    if(navigation.granted)navigation.cancel(NavigationLease::NAV_TAKEOVER);
    if (state != STATE_BALANCING) { Serial.println("[GEST] Not balancing -- ignored"); return; }
    if (climbMode || autoClimb.targetClimb()) { Serial.println("[GEST] Climb mode -- gestures ignored"); return; }
    gestSeq = seq; gestLen = len; gestIdx = 0; gestStepStart = millis();
    Serial.printf("[GEST] %s\n", name);
}
void stopGesture() { gestSeq = nullptr; }

void requestEnable() {
    if (!startupGate.mayArm(P.bootWait > 0.5f)) {
        btgamepad_syncArmed(false);
        Serial.println("[MAIN] Startup is not ready; press START after calibration completes");
        return;
    }
    motorsRequested = true;
    Serial.println("[MAIN] Enable requested — will arm when upright");
}

void requestDisable(const char* why) {
    navigation.cancel(NavigationLease::NAV_DISABLED);radxaLastRx=0;radxaFwd=radxaSteer=0;
    motorsRequested = false;
    // Push the state back into the pad module. Without this, an AUTOMATIC
    // disable (FELL / RATE SPIKE) left the pad still believing it was armed,
    // so the next START press produced no edge and appeared to do nothing.
    btgamepad_syncArmed(false);
    if (state == STATE_BALANCING || hub.isEnabled()) {
        hub.disable();
        state = STATE_IDLE;
    }
    wheelTargetL=wheelTargetR=appliedSteerSteps=0.0f;
    autoClimb.resetDynamics();climbMode=false;
    Serial.printf("[MAIN] Motors OFF (%s)\n", why);
}

// ============================================================
//  SERIAL PARSER — Radxa 'V' lines + tuner commands, one reader
// ============================================================
void handleLine(char* line) {
    if(!strcmp(line,"FU,1") || !strcmp(line,"FU,0")) {
        if(line[3]=='1' && (state==STATE_BALANCING || motorsRequested)) {
            Serial.println("FU?,BUSY");return;
        }
        startupGate.maintenance=line[3]=='1';
        if(startupGate.maintenance) {
            if(hardwareReady) { char stop[]="AT,stop";at_handleLine(stop);requestDisable("firmware maintenance"); }
            else motorsRequested=false;
        }
        Serial.printf("FU!,%u\n",(unsigned)startupGate.maintenance);
        return;
    }
    if (!strcmp(line, "BR,1")) {
        hostBootReady = true;
        startupGate.acknowledge(true);
        Serial.println("BR!,1");
        if (startupGate.initialized) Serial.println("BS,READY");
        return;
    }
    if (!strcmp(line, "BR,0")) {
        hostBootReady = false;
        startupGate.acknowledge(false);
        if (hardwareReady) requestDisable("Radxa boot wait");
        else motorsRequested = false;
        Serial.println("BR?,0");
        return;
    }

    // Keep discovery alive while all sensor/controller startup is on HOLD.
    // In particular P? -> BOOTGATE is how Radxa knows to send BR,1.
    if (!startupGate.initialized) {
        if (!strcmp(line, "P?") || !strcmp(line, "p?")) { params_dump(); return; }
        if (!strcmp(line, "X") || !strcmp(line, "x")) { motorsRequested = false; return; }
        Serial.println("[BOOT] Startup on hold/in progress; only BR and P? are accepted");
        return;
    }

    if(startupGate.maintenance && !strncmp(line,"AT,",3) && strcmp(line,"AT,stop") && strcmp(line,"AT,?")) {
        Serial.println("[MAIN] Auto-tune blocked during firmware maintenance");return;
    }
    if (at_handleLine(line)) return;      // AT,<routine>, only after startup
    if (params_handleLine(line)) return;

    if(!strncmp(line,"AN,",3)) {
        unsigned long owner=0;int en=0;char extra=0;
        if(sscanf(line+3,"%lu,%d%c",&owner,&en,&extra)!=2 || (en!=0 && en!=1))return;
        if(!en) { if(navigation.granted && navigation.token==owner)navigation.cancel(NavigationLease::NAV_STOPPED);return; }
        const bool neutral=fabsf(joyForward*P.joyFwdSc)<.10f && fabsf(joySteering*P.joyStrSc)<.10f;
        const bool available=!startupGate.maintenance && btgamepad_connected() &&
                             state==STATE_BALANCING;
        if(navigation.grant((uint32_t)owner,millis(),state==STATE_BALANCING,available,neutral)) {
            stopGesture();radxaLastRx=0;Serial.printf("AN!,%lu\n",owner);
        } else Serial.printf("AN?,%lu\n",owner);
        return;
    }
    if(!strncmp(line,"AU,",3)) {
        unsigned long owner=0,seq=0;float v=0,w=0;char extra=0;
        if(sscanf(line+3,"%lu,%lu,%f,%f%c",&owner,&seq,&v,&w,&extra)==4)
            navigation.command((uint32_t)owner,(uint32_t)seq,v,w,millis());
        return;
    }

    if (!strcmp(line,"AC,1") || !strcmp(line,"AC,0")) {
        if(navigation.granted)navigation.cancel(NavigationLease::NAV_TAKEOVER);
        autoClimb.configure(line[3]=='1');stiffHold=false;
        Serial.printf("[AUTOCLIMB] %s\n",autoClimb.enabled?"AUTO ENABLED":"AUTO DISABLED");
        return;
    }

    if (!strncmp(line, "TG,", 3)) {
        int en = 0; float limit = 0; char extra = 0;
        if (sscanf(line + 3, "%d,%f%c", &en, &limit, &extra) != 2 ||
            !tofGuard.update(en, limit, millis()))
            Serial.println("[TOF] Invalid TG command");
        return;
    }

    if (line[0] == 'V' && line[1] == ',') {
        float f = 0, s = 0; int en = 0;
        if (sscanf(line + 2, "%f,%f,%d", &f, &s, &en) == 3 && isfinite(f) && isfinite(s)) {
            radxaFwd   = clampf(f, -1.0f, 1.0f);
            radxaSteer = clampf(s, -1.0f, 1.0f);
            radxaEn    = en ? 1 : 0;
            radxaLastRx = millis();
            // A remote link may always STOP the robot; it may never START it.
            // Arming stays a deliberate local action (pad START, or 'E' on
            // USB serial). Without this, the Cubie's very first V-line --
            // including the vision guard's own "V,0,0,1" -- armed the robot.
            if (!radxaEn && radxaEnPrev)  requestDisable("Radxa disable");
            radxaEnPrev = radxaEn;
        }
        return;
    }

    // ---- expression commands (from the phone app / Radxa) ----
    if ((line[0] == 'G' || line[0] == 'g') && line[1] == ',') {
        const char* n = line + 2;
        if      (!strncmp(n, "yes",   3)) startGesture(GEST_YES,   sizeof(GEST_YES)/sizeof(GStep),   "nod YES");
        else if (!strncmp(n, "no",    2)) startGesture(GEST_NO,    sizeof(GEST_NO)/sizeof(GStep),    "nod NO");
        else if (!strncmp(n, "spin",  4)) startGesture(GEST_SPIN,  sizeof(GEST_SPIN)/sizeof(GStep),  "spin");
        else if (!strncmp(n, "dance", 5)) startGesture(GEST_DANCE, sizeof(GEST_DANCE)/sizeof(GStep), "dance");
        else if (!strncmp(n, "stop",  4)) { stopGesture(); Serial.println("[GEST] stop"); }
        else Serial.printf("[GEST] Unknown: %s\n", n);
        return;
    }
    if ((line[0] == 'H' || line[0] == 'h') && line[1] == ',') {
        if(navigation.granted)navigation.cancel(NavigationLease::NAV_TAKEOVER);
        int hv = atoi(line + 2);
        if (hv == 2) {                                  // CLIMB mode
            autoClimb.select(AutoClimb::FORCE_CLIMB);stiffHold=false;stopGesture();
            Serial.println("[HOLD] CLIMB");
        } else if(hv==3) {
            autoClimb.configure(true);stiffHold=false;
            Serial.println("[AUTOCLIMB] AUTO ENABLED");
        } else {
            autoClimb.select(hv==0?AutoClimb::FORCE_NORMAL:AutoClimb::FORCE_STIFF);
            zInt = clampf(zInt, -P.zIntLim, P.zIntLim); // re-shrink climb integral
            stiffHold = hv != 0;
            if (stiffHold) { xRef = posM; zInt = 0.0f; }   // hold RIGHT HERE
            Serial.printf("[HOLD] %s\n", stiffHold ? "STIFF" : "normal");
        }
        return;
    }
    if ((line[0] == 'A' || line[0] == 'a') && line[1] == ',') {
        lookCmd = clampf(atof(line + 2), -1.0f, 1.0f);
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
        else if (cmd == 'M') { params_set("CURRENT", val); }
        else if (cmd == 'B') { payload_torchSetPct((int)val); }
        else if (cmd == 'P') { payload_setYaw(val);  Serial.printf("[PAY] Pan  -> %.0f deg\n", payload_yaw());  }
        else if (cmd == 'Z') { payload_setZoom(val); Serial.printf("[PAY] Zoom -> %.0f deg\n", payload_zoom()); }
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
        case 'F': payload_torchToggle(); break;
        case 'N': payload_center(); break;
        case 'L': DEBUG_STREAM = !DEBUG_STREAM; break;
        case 'S':
            if (params_motionActive()) { Serial.println("[HUB] Use HM telemetry while balancing; stop for settings"); break; }
            Serial.println("\n---- Package B (LQR / LQI) settings ----");
            Serial.printf("  K1=%.4f  K2=%.4f  K3=%.4f\n", k1, k2, k3);
            Serial.printf("  K4=%.4f  K5=%.4f  Trim=%.3f deg\n", k4, k5, pitchTrim);
            Serial.printf("  Motors: %s  State: %s  PadSpeed: %s  Hold: %s\n",
                          hub.isEnabled() ? "ON" : "OFF",
                          state == STATE_BALANCING ? "BALANCING" : "IDLE",
                          joySpeedHigh ? "HIGH" : "LOW",
                          climbMode ? "CLIMB" : (stiffHold ? "STIFF" : "normal"));
            Serial.printf("  Payload: pan=%.0f deg  zoom=%.0f deg  torch=%s %d%% (cap %d%%)\n",
                          payload_yaw(), payload_zoom(),
                          payload_torchIsOn() ? "ON" : "off",
                          payload_torchPct(), TORCH_MAX_PCT);
            hub.printStatus();
            Serial.println("----------------------------------------\n");
            break;
        case '?':
            Serial.println("\nE enable | X stop | C cal gyro | S settings | L debug | R reset");
            Serial.println("K1=..K5= LQR gains   T= trim (deg)   M= motor current (mA)");
            Serial.printf ("F torch toggle | N centre servos | B=<%d..%d> torch %%%% | P=<deg> pan | Z=<deg> zoom\n",
                           TORCH_MIN_PCT, TORCH_MAX_PCT);
            Serial.println("G,yes|no|spin|dance|stop   H,0|1|2|3 normal|stiff|CLIMB|auto   AC,0|1 auto off|on   A,<-1..1> look");
            Serial.println("Radxa: V,<fwd -1..1>,<steer -1..1>,<en 0|1>\n");
            break;
        default: break;
    }
}

void pollSerial() {
    uint16_t budget = 192;
    while (budget-- && Serial.available()) {
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

    // Preserve terrain adaptation during calm motion; manoeuvre/recovery
    // feedback takes priority when the reference governor detects a hazard.
    const bool  airborne = terrain_airborne() && P.airFreeze > 0.5f;
    const float driveScale = P.maxDriveV+autoClimb.referenceBlend*(P.climbVel-P.maxDriveV);
    float nominalDriveV=navigation.granted?navigation.velocity:cmdFwd*driveScale;
    requestedDriveV = navigation.granted?nominalDriveV:descentGuard.limitReference(nominalDriveV);
    fallPrevention.update(requestedDriveV,velF,pitchF,rateF,vCmd,P.vMax,P.aMax,P.maxTilt,dt,airborne);
    // A tighter obstacle/heartbeat reference cap takes effect immediately,
    // rather than waiting for the normal operator-reference slew to catch up.
    if (!navigation.granted && tofGuard.enabled) {
        const float boundedRef = driveScale>0.0f ?
            tofGuard.apply(fallPrevention.operatorRef/driveScale,driveScale,millis())*driveScale : 0.0f;
        fallPrevention.constrainForwardReference(boundedRef,velF);
    }
    const bool wasAutomatic=autoClimb.active;
    const bool wasLimited=autoClimb.actuatorLimited;
    const bool wasExiting=autoClimb.exitPending;
    const float wheelDiffVelocity=diffRateF/(2.0f*P.countsPerM);
    // Observe the nominal reference, not the speed reduction this detector
    // itself created. Raw intent also resets confirmation during stick changes.
    autoClimb.update(nominalDriveV,velF,pitchF,rateF,cmdSteer,
                     wheelTargetL/P.stepsPerM,wheelTargetR/P.stepsPerM,
                     velF-wheelDiffVelocity,velF+wheelDiffVelocity,P.maxDriveV,
                     fallPrevention.hazardRisk,terrain_airborne(),stiffHold || gestSeq!=nullptr,satA!=0,dt,
                     navigation.granted?nominalDriveV:cmdFwd*P.maxDriveV);
    const bool descentEligible=(autoClimb.enabled && autoClimb.policy==AutoClimb::AUTOMATIC) ||
                               autoClimb.policy==AutoClimb::FORCE_CLIMB;
    const bool descentCue=(autoClimb.active && autoClimb.kind==AutoClimb::DOWN_LOAD) ||
                          (autoClimb.candidateKind==AutoClimb::DOWN_LOAD && autoClimb.suspectSeconds>=.25f) ||
                          (autoClimb.candidateKind==AutoClimb::DOWN_LOAD && autoClimb.overspeed>.35f) ||
                          (autoClimb.policy==AutoClimb::FORCE_CLIMB && autoClimb.overspeed>.18f);
    descentGuard.update(navigation.granted?nominalDriveV:cmdFwd*P.climbVel,velF,
                        descentEligible,descentCue,terrain_airborne(),dt);
    if(autoClimb.exitPending && !wasExiting)Serial.println("[AUTOCLIMB] EXIT CHECK: monitored normal-controller hand-back");
    if(!autoClimb.exitPending && wasExiting && autoClimb.active)
        Serial.println("[AUTOCLIMB] EXIT CANCELLED: load/overspeed still requires compensation");
    if(autoClimb.active!=wasAutomatic)
        Serial.printf("[AUTOCLIMB] %s %s; error=%.3f m/s bias=%.2f deg\n",
                      autoClimb.active?"ENTER":"EXIT",autoClimb.kind==AutoClimb::DOWN_LOAD?"DOWNCLIMB":"load inferred",
                      autoClimb.trackingError,autoClimb.pitchBias);
    if(autoClimb.actuatorLimited && !wasLimited)
        Serial.println("[AUTOCLIMB] LIMITED: wheel command/encoder mismatch; check load/driver/supply");
    climbMode=autoClimb.blend>.001f || autoClimb.active; // retain shadow C state during exit verification
    float vIn = fallPrevention.effectiveRef;
    const float deadband = P.driveDb*P.maxDriveV;
    if(fabsf(requestedDriveV)>deadband || fabsf(fallPrevention.operatorRef)>deadband)
        driveReferenceActive=true;
    if(fabsf(requestedDriveV)<=deadband && fabsf(fallPrevention.operatorRef)<=deadband &&
       fabsf(velF)<.05f && !fallPrevention.recovering) driveReferenceActive=false;
    bool driving = driveReferenceActive;
    const float normalScale=(fallPrevention.fullBalanceFeedback || (stiffHold && !driving)) ?
                            1.0f:terrain_soften();
    const float sf=normalScale+autoClimb.blend*(1.0f-normalScale);
    appliedBalanceScale = sf;
    const float k3t = k3*sf, k4t = k4*sf;

    if (driving) {
        // Velocity-tracking mode: K2..K4 subset (poles verified stable).
        // Hold point is dragged along with a braking lookahead so the
        // robot parks smoothly where it stops.
        xRef = posM + velF * P.brakeLook;
        zInt = 0.0f;
        uOut = -(k2 * (velF - vIn) + k3t * th + k4t * om);
    } else {
        if(fallPrevention.recovering) {
            xRef += (posM-xRef)*clampf(2.0f*fallPrevention.risk*dt,0.0f,1.0f);
            zInt -= zInt*clampf(2.0f*fallPrevention.risk*dt,0.0f,1.0f);
        }
        // Position hold / go-to: full LQI vector. Position error is
        // clamped so a big displacement asks for a gentle walk back,
        // never a sprint that eats the balance headroom.
        float ex = clampf(posM - xRef, -P.exClamp, P.exClamp);
        if (!airborne) zInt = clampf(zInt + ex * dt, -P.zIntLim, P.zIntLim);
        if (stiffHold) {
            uOut = -(P.s1 * ex + P.s2 * velF + P.s3 * th + P.s4 * om + P.s5 * zInt);
        } else {
            uOut = -(k1 * ex + k2 * velF + k3t * th + k4t * om + k5 * zInt);
        }
    }

    // Separate climb state makes entry/exit a 0.75 s convex blend instead of
    // replacing the normal position reference/integral in one control cycle.
    if(climbMode) {
        if(!climbReferenceInitialized) {
            climbXRef=posM;
            climbDriveRef=clampf(velF,-P.climbVel,P.climbVel);
            climbIntegral=fabsf(P.c5)>.0001f ?
                clampf((-uOut-P.c2*velF-P.c3*th-P.c4*om)/P.c5,-P.climbZLim,P.climbZLim) : 0;
            climbReferenceInitialized=true;
        }
        const float climbGoal=navigation.granted?nominalDriveV:descentGuard.goal;
        climbDriveRef+=clampf(climbGoal-climbDriveRef,-2.40f*dt,2.40f*dt);
        if(!navigation.granted && tofGuard.enabled && P.climbVel>0) {
            const float cap=tofGuard.apply(climbDriveRef/P.climbVel,P.climbVel,millis())*P.climbVel;
            climbDriveRef=fminf(climbDriveRef,cap);
        }
        if(airborne)climbDriveRef=0;
        climbRecovery=fallPrevention.climbRecoveryScale();
        climbXRef+=climbDriveRef*(1.0f-climbRecovery)*dt;
        if(climbRecovery>0) {
            const float bleed=clampf(2.0f*climbRecovery*dt,0,1);
            climbXRef+=(posM-climbXRef)*bleed;climbIntegral-=climbIntegral*bleed;
        }
        climbXRef=clampf(climbXRef,posM-P.exClamp,posM+P.exClamp);
        const float climbError=posM-climbXRef;
        const float dz=airborne || autoClimb.blend<.05f?0.0f:climbError*dt;
        const float trial=clampf(climbIntegral+dz,-P.climbZLim,P.climbZLim);
        const float trialU=-(P.c1*climbError+P.c2*velF+P.c3*th+P.c4*om+P.c5*trial);
        // Conditional anti-windup: only reject integral motion that would push
        // the selected acceleration deeper into its physical clamp.
        const float integralEffect=-P.c5*(trial-climbIntegral);
        const float mixedTrial=uOut+autoClimb.blend*(trialU-uOut);
        if(!((mixedTrial>P.aMax && integralEffect>0) || (mixedTrial<-P.aMax && integralEffect<0)))
            climbIntegral=trial;
        const float climbU=-(P.c1*climbError+P.c2*velF+P.c3*th+P.c4*om+P.c5*climbIntegral);
        uOut+=autoClimb.blend*(climbU-uOut);
    } else {
        climbReferenceInitialized=false;climbXRef=posM;climbIntegral=climbDriveRef=climbRecovery=0;
    }

    satA = (fabsf(uOut) > P.aMax) ? 1 : 0;
    uOut = clampf(uOut, -P.aMax, P.aMax);
    const auto velocity = hub_control::integrateVelocity(vCmd,uOut,velF,dt,P.hubKp,P.motorMa*.001f,P.vMax);
    vCmd = velocity.value;
    satV = velocity.trackingLimited;

    // BALANCE > POSITION: if the speed command saturates, the tilt
    // terms lose their actuator and the robot falls. Give up ground:
    // bleed the hold point (and integral) toward the robot until
    // authority returns. It parks a little off-spot instead of falling.
    if (satV || hub.saturated() || (P.vMax>0 && fabsf(vCmd) > 0.90f * P.vMax)) {
        xRef += (posM - xRef) * clampf(P.vsatBleed * dt, 0.0f, 1.0f);
        zInt -= zInt * clampf(2.0f * dt, 0.0f, 1.0f);
        if(climbMode) {
            climbXRef+=(posM-climbXRef)*clampf(P.vsatBleed*dt,0,1);
            climbIntegral-=climbIntegral*clampf(2.0f*dt,0,1);
        }
    }

    float baseSteps = vCmd * P.stepsPerM;

    // ---- yaw: commanded turn or encoder-differential heading hold ----
    float steerIn = cmdSteer*(1.0f+autoClimb.blend*(P.climbStr-1.0f));
    const bool navigationTurning=navigation.granted && fabsf(navigation.yawRate)>.005f;
    const bool turning = navigationTurning || fabsf(steerIn) > P.steerDb;
    const float target = navigationTurning ? navigation.yawRate*P.trackW*P.stepsPerM*.5f : steerIn*P.maxSteerSt;
    const float room = P.maxSpdEff>0 ? fmaxf(0,P.maxSpdEff-fabsf(baseSteps)) : P.maxSteerSt;
    steerSteps = yawControl.update(clampf(target,-P.maxSteerSt,P.maxSteerSt),turning,encDiff,diffRateF,
        lookCmd*(P.lookMaxDeg*DEG_TO_RAD)*P.trackW*P.countsPerM,P.yawKp,P.yawKd,P.yawMaxSt,
        P.yawRelatch,fabsf(pitchF)>P.yawTiltSus,P.steerSlew,room,.1f*P.trackW*P.countsPerM,dt);
    diffTarget = yawControl.heading;
    appliedSteerSteps = fallPrevention.allocateSteering(baseSteps,steerSteps,P.maxSpdEff);
    wheelTargetL=baseSteps-appliedSteerSteps;
    wheelTargetR=baseSteps+appliedSteerSteps;
    const bool recoveryTorque = fallPrevention.recovering || fabsf(pitchF)>=6.0f || fabsf(rateF)>=40.0f;
    hub.setBalanceCommand(wheelTargetL,wheelTargetR,uOut,recoveryTorque);
}

// ============================================================
//  SETUP
// ============================================================
void waitForHostStartup() {
    if (P.bootWait <= 0.5f) {
        Serial.println("BS,INITIALIZING");
        Serial.println("[BOOT] BOOTWAIT disabled: explicit standalone startup");
        return;
    }
    Serial.println("BS,WAIT_HOST");
    Serial.println("[BOOT] Waiting for Radxa ACK; IMU calibration and device startup are on HOLD");
    uint32_t lastRequest = millis();
    while (!startupGate.mayInitialize(P.bootWait > 0.5f)) {
        pollSerial();
        params_service();
        if (millis() - lastRequest >= 1000) {
            Serial.println("BR?,0");
            lastRequest = millis();
        }
        delay(5);  // yield while waiting; no timeout that starts calibration prematurely
    }
    Serial.println("BS,INITIALIZING");
    Serial.println("[BOOT] Radxa ACK received; starting deferred initialization");
}

void setup() {
    pinMode(RIGHT_RS485_DE,OUTPUT); digitalWrite(RIGHT_RS485_DE,LOW);
    pinMode(LEFT_RS485_DE,OUTPUT); digitalWrite(LEFT_RS485_DE,LOW);
    digitalWrite(TORCH_PIN, LOW);
    pinMode(TORCH_PIN, OUTPUT);
    Serial.setTxBufferSize(1024);
    Serial.begin(SERIAL_BAUD);
    delay(300);
    // GPIO 2 is the torch MOSFET gate now; payload_begin() claims it
    // below and drives it low. The WS2812 ring shows status instead.

    Serial.println("\n==================================================");
    Serial.println("     2PyBot BaseLink v4.0.2 — M0601 BLDC / LQI");
    Serial.println("==================================================");

    params_begin();
    Serial.println("BR?,0");
    params_dump(); // capability/readback only; does not access IMU or drivers
    waitForHostStartup();
    joySpeedHigh = (P.bootHigh > 0.5f) ? 1 : 0;     // NVS or compiled defaults, before any consumer
    leds_begin();   // boot swirl plays while sensors settle below

    if (!imu.begin()) {
        Serial.println("BS,ERROR");
        Serial.println("[MAIN] FATAL: IMU init failed — halting");
        while (true) delay(1000);
    }
    if (!hub.begin()) {
        Serial.println("[MAIN] Hub link unavailable; arming remains blocked");
    }

    Serial.printf("[MAIN] Settling sensors %d ms — keep robot still\n", STARTUP_SETTLE_MS);
    Serial.println("BS,CALIBRATING");
    delay(STARTUP_SETTLE_MS);
    if (!imu.calibrateGyro()) {
        Serial.println("BS,ERROR");
        Serial.println("[MAIN] FATAL: gyro calibration failed — motors remain disabled");
        while (true) delay(1000);
    }

    btgamepad_begin();
    payload_begin();
    hardwareReady = true;
    // Consume commands buffered during initialization while the startup guard is
    // still closed. An old serial enable must not arm immediately after calibration.
    pollSerial();
    startupGate.complete();
    motorsRequested = false;
    btgamepad_syncArmed(false);

    Serial.printf("[MAIN] P.stepsPerM=%.0f  P.countsPerM=%.0f  Vmax=%.2f m/s  Amax=%.2f m/s^2\n",
                  P.stepsPerM, P.countsPerM, P.vMax, P.aMax);
    Serial.println("BS,READY");
    Serial.println("[MAIN] Calibration complete; place upright and press local START or serial E to arm.\n");

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
    if (elapsed > loopMaxUs) loopMaxUs = elapsed;
    float dt = elapsed / 1000000.0f;
    if (dt > 0.05f) dt = 0.05f;               // guard after stalls

    // ---- 1. sensors ----
    imu.update(dt);
    pitchF = P.pitchSign * (imu.getPitch() - pitchTrim);
    // road-condition estimate: |accel| deviation from 1 g + rate spikes
    terrain_update(sqrtf(imu.getAx()*imu.getAx() + imu.getAy()*imu.getAy() +
                         imu.getAz()*imu.getAz()), rateF, dt);
    rateF  = P.rateSign  *  imu.getPitchRate();

    hub.service(dt);
    encRawL = hub.getPositionL();
    encRawR = hub.getPositionR();
    float posL = P.encSign * (float)encRawL;
    float posR = P.encSign * (float)encRawR;

    static float posPrev = 0.0f; static bool posInit = false;
    posM = 0.5f * (posL + posR) / P.countsPerM;
    if (!posInit) { posPrev=posM;posInit=true; }
    float vRaw = (posM-posPrev)/dt;
    posPrev=posM;
    const float aV = 1.0f - expf(-2.0f * PI * 15.0f * dt);
    velF += aV * (vRaw - velF);

    static float diffPrev = 0.0f; static bool diffInit = false;
    encDiff = posR - posL;
    if (!diffInit) { diffPrev = encDiff; diffInit = true; }
    float dRaw = (encDiff - diffPrev) / dt;
    diffPrev = encDiff;
    diffRateF += aV * (dRaw - diffRateF);

    // ---- 2. inputs ----
    pollSerial();
    btgamepad_update();   // fills joyForward/joySteering/joyEnable (same names as before)

    // Gamepad expression buttons -> existing gesture engine (edge-detected in bt_gamepad.h)
    switch (btgamepad_takeGesture()) {
        case 1: startGesture(GEST_YES,   sizeof(GEST_YES)/sizeof(GStep),   "nod YES"); break;
        case 2: startGesture(GEST_NO,    sizeof(GEST_NO)/sizeof(GStep),    "nod NO");  break;
        case 3: startGesture(GEST_SPIN,  sizeof(GEST_SPIN)/sizeof(GStep),  "spin");    break;
        case 4: startGesture(GEST_DANCE, sizeof(GEST_DANCE)/sizeof(GStep), "dance");   break;
        case 5: stiffHold = !stiffHold;
                if(navigation.granted)navigation.cancel(NavigationLease::NAV_TAKEOVER);
                autoClimb.select(stiffHold?AutoClimb::FORCE_STIFF:AutoClimb::FORCE_NORMAL);
                if (stiffHold) { xRef = posM; zInt = 0.0f; }
                Serial.printf("[HOLD] %s (pad)\n", stiffHold ? "STIFF" : "normal");
                break;
        case 6: {
                if(navigation.granted)navigation.cancel(NavigationLease::NAV_TAKEOVER);
                const bool selectClimb=!autoClimb.targetClimb();
                autoClimb.select(selectClimb?AutoClimb::FORCE_CLIMB:AutoClimb::FORCE_NORMAL);
                stiffHold=false;stopGesture();
                Serial.printf("[HOLD] %s (pad; manual override)\n",selectClimb?"CLIMB":"climb off");
                break;
                }
        default: break;
    }

    // ---- 2b. camera payload: right stick pans/zooms, RB torch,
    //          D-pad LEFT/RIGHT dims. Deliberately OUTSIDE the state
    //          machine: the camera and light stay usable whether the
    //          robot is balancing, idle, or lying on its side.
    if (btgamepad_takeTorchToggle()) payload_torchToggle();
    {
        int8_t tStep = btgamepad_takeTorchStep();
        if (tStep) payload_torchStep(tStep);
    }
    payload_update(dt, joyPanX, joyZoomY);


    // Speed-mode edge -> ring cue (blue dash = HIGH, green settle = LOW)
    static uint8_t spdPrev = SPEED_BOOT_HIGH;
    if (joySpeedHigh != spdPrev) {
        spdPrev = joySpeedHigh;
        leds_event(spdPrev ? LED_EV_SPEED_HI : LED_EV_SPEED_LO);
    }

    // Gamepad enable edge (works even when Radxa has drive authority)
    // Enable/disable on real button PRESSES, not on level transitions.
    // joyEnable boots at 0, so the old "falling edge" test could never fire
    // before START had been pressed once -- SELECT was a no-op from boot.
    int8_t enEv = btgamepad_takeEnableEvent();
    if (enEv > 0) requestEnable();
    if (enEv < 0) requestDisable("joystick e-stop");

    // Drive-authority arbitration: Radxa fresh > joystick fresh > zero
    bool radxaFresh = (millis() - radxaLastRx) < P.radxaTmo && radxaLastRx != 0;
    bool joyFresh   = (millis() - lastJoyPacketMs) < P.joyTmo && lastJoyPacketMs != 0;
    const float manualFwd=clampf(joyForward*P.joyFwdSc,-1.0f,1.0f)*(joySpeedHigh?1.0f:P.spdLoDrv);
    const float manualSteer=clampf(joySteering*P.joyStrSc,-1.0f,1.0f)*(joySpeedHigh?1.0f:P.spdLoStr);
    const bool takeover=joyFresh && (fabsf(joyForward*P.joyFwdSc)>.12f || fabsf(joySteering*P.joyStrSc)>.18f);
    const NavigationLease::State previousNavState=navigation.state;
    navigation.update(millis(),takeover,state==STATE_BALANCING && !startupGate.maintenance && btgamepad_connected());
    if(previousNavState!=navigation.state && navigation.token)
        Serial.printf("[NAV] state=%u token=%lu\n",(unsigned)navigation.state,(unsigned long)navigation.token);
    if(navigation.granted) {
        const float scale=P.maxDriveV+autoClimb.referenceBlend*(P.climbVel-P.maxDriveV);
        cmdFwd=scale>0?clampf(navigation.velocity/scale,-1,1):0;
        cmdSteer=clampf(navigation.yawRate*P.trackW*P.stepsPerM/(2.0f*P.maxSteerSt),-1,1);
    } else if(navigation.token) {
        // A cancelled session latches manual priority; stale host commands may
        // not resume driving when the operator centres the stick again.
        cmdFwd=joyFresh?manualFwd:0;cmdSteer=joyFresh?manualSteer:0;
    } else if (radxaFresh) {
        cmdFwd = radxaFwd;  cmdSteer = radxaSteer;
    } else if (joyFresh) {
        // Dual speed mode (LB = LOW, RB = HIGH): scales the pad's authority
        // INSIDE the existing caps. Full stick in LOW mode commands
        // P.spdLoDrv * P.maxDriveV and
        // P.spdLoStr * P.maxSteerSt. Radxa override (above)
        // and gesture keyframes (below) stay full-scale on purpose, and
        // the balancer's own limits (P.vMax / P.aMax) never shrink.
        float sD = joySpeedHigh ? 1.0f : P.spdLoDrv;
        float sS = joySpeedHigh ? 1.0f : P.spdLoStr;
        cmdFwd   = clampf(joyForward * P.joyFwdSc,  -1.0f, 1.0f) * sD;
        cmdSteer = clampf(joySteering * P.joyStrSc, -1.0f, 1.0f) * sS;
    } else {
        cmdFwd = 0.0f; cmdSteer = 0.0f;
    }

    // ---- 2b. gesture playback (operator input always overrides) ----
    if (gestSeq) {
        if (fabsf(cmdFwd) > 0.10f || fabsf(cmdSteer) > 0.20f) {
            stopGesture();                        // human grabbed the stick
        } else {
            if (millis() - gestStepStart >= gestSeq[gestIdx].ms) {
                gestIdx++; gestStepStart = millis();
                if (gestIdx >= gestLen) stopGesture();
            }
            if (gestSeq) { cmdFwd = gestSeq[gestIdx].fwd; cmdSteer = gestSeq[gestIdx].steer; }
        }
    }

    // Optional ToF constraint applies to EVERY drive source, including gestures.
    // Reverse/steering and balance-recovery authority are unchanged.
    // The optional manual-drive ToF guard does not gate autonomous routes.
    if(!navigation.granted)
        cmdFwd = tofGuard.apply(cmdFwd,P.maxDriveV+autoClimb.referenceBlend*(P.climbVel-P.maxDriveV),millis());

    // ---- 3. state machine ----
    if (state == STATE_IDLE) {
        if (motorsRequested && hub.linkOk(HUB_LINK_TIMEOUT_MS) && hub.modeOk() && !hub.faultBits() &&
            P.drvMode>.5f && !startupGate.maintenance && (P.bootWait < 0.5f || hostBootReady) && fabsf(pitchF) < P.armAngle) {
            resetController();
            hub.enable();
            state = STATE_BALANCING;
            leds_event(LED_EV_ARM);
            Serial.println("[MAIN] -> BALANCING");
        }
    } else { // BALANCING
        if (!hub.linkOk(HUB_LINK_TIMEOUT_MS) || !hub.modeOk() || hub.faultBits() || elapsed>20000) {
            static char reason[160];
            const auto& l = hub.left().fb(); const auto& r = hub.right().fb();
            snprintf(reason, sizeof(reason),
                "%s: link=%u mode=%u fault L0x%02X R0x%02X modeL%u modeR%u ageL%lums ageR%lums toL%lu toR%lu crcL%lu crcR%lu",
                elapsed>20000 ? "CONTROL LOOP OVERRUN" : "MOTOR LINK/MODE/FAULT",
                (unsigned)hub.linkOk(HUB_LINK_TIMEOUT_MS), (unsigned)hub.modeOk(), l.fault, r.fault, l.mode, r.mode,
                (unsigned long)(millis()-l.rxMs), (unsigned long)(millis()-r.rxMs),
                (unsigned long)l.timeouts, (unsigned long)r.timeouts,
                (unsigned long)l.crcErrors, (unsigned long)r.crcErrors);
            requestDisable(reason);
        } else if (fabsf(pitchF) > P.maxTilt || fabsf(rateF) > P.maxRate) {
            leds_event(LED_EV_FALL);
            requestDisable(fabsf(pitchF) > P.maxTilt ? "FELL" : "RATE SPIKE");
        } else {
            runController(dt);
        }
    }

    if (AT.mode != AT_IDLE && (!hub.linkOk(HUB_LINK_TIMEOUT_MS) || !hub.modeOk() || hub.faultBits() || elapsed>20000))
        at_restore("motor feedback/control timing lost");
    at_update(dt);
    hub.transmit(dt);       // both UARTs immediately after calculating control
    params_service();      // incremental output, never blocks motor updates

    // ---- 3b. expression ring (rate-limited internally, non-blocking) ----
    {
        LedInputs li;
        li.balancing    = (state == STATE_BALANCING);
        li.armRequested = motorsRequested;
        li.fwd          = cmdFwd;
        li.steer        = cmdSteer;
        li.velF         = velF;
        li.pitchF       = pitchF;
        li.ex           = posM - (autoClimb.blend>=.5f?climbXRef:xRef);
        li.stiffHold    = stiffHold;
        li.climbMode    = climbMode;
        li.gestActive   = (gestSeq != nullptr);
        li.radxaFresh   = radxaFresh;
        li.padConnected = btgamepad_connected();
        leds_update(li);
    }

    // ---- 4. odometry stream to Radxa, 50 Hz, always on ----
    uint32_t nowMs = millis();
    static uint32_t lastNavMs=0;
    if(nowMs-lastNavMs>=100 && Serial.availableForWrite()>=96) {
        lastNavMs=nowMs;
        Serial.printf("NR,%lu,%lu,%u,%lu,%lu,%u,%u\n",(unsigned long)nowMs,(unsigned long)navigation.token,
                      (unsigned)navigation.state,(unsigned long)navigation.sequence,(unsigned long)(nowMs-navigation.lastRx),
                      (unsigned)btgamepad_connected(),0u);
    }
    if (nowMs - lastOdomMs >= ODOM_PERIOD_MS && RADXA_STREAM && Serial.availableForWrite()>=96) {
        lastOdomMs = nowMs;
        Serial.printf("O,%lu,%lld,%lld,%.2f,%.2f\n",
                      (unsigned long)nowMs,
                      (long long)encRawL, (long long)encRawR,
                      imu.getPitch(), imu.getYaw());
    }

    // ---- 5. optional controller debug, nominal 50 Hz ----
     if (DEBUG_STREAM && nowMs - lastDbgMs >= DEBUG_PERIOD_MS && Serial.availableForWrite()>=640) {
        lastDbgMs = nowMs;
        // D,ms,pF,w,u,v,ex,vCmdSteps,steerSteps,satV,satA,loopMaxUs,speedHi,state,fwd,str
        Serial.printf("D,%lu,%.2f,%.1f,%.2f,%.3f,%.3f,%.0f,%.0f,%u,%u,%lu,%u,%u,%.2f,%.2f,"
                       "%.3f,%.3f,%.2f,%u,%lu,%.3f,%.2f,%u,%.3f,%.3f,%.3f,%.2f,%.0f,%.0f,"
                       "%u,%u,%u,%.2f,%.3f,%.2f,%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
                       "%u,%.2f,%u,%.2f,%.3f,%.3f,%.3f,%u,%.3f,%.3f\n",
                       (unsigned long)nowMs, pitchF, rateF, uOut, velF,posM-(autoClimb.blend>=.5f?climbXRef:xRef),
                       vCmd * P.stepsPerM, appliedSteerSteps, satV, satA,
                      (unsigned long)loopMaxUs, (unsigned)joySpeedHigh,
                      (unsigned)(state == STATE_BALANCING), cmdFwd, cmdSteer,
                      TERR.accelMag, TERR.rough, TERR.soften,
                      (unsigned)TERR.state, (unsigned long)TERR.drops,
                      fallPrevention.risk,fallPrevention.predictedTilt,(unsigned)fallPrevention.reasons,
                      requestedDriveV,fallPrevention.operatorRef,fallPrevention.effectiveRef,
                       appliedBalanceScale,wheelTargetL,wheelTargetR,
                       (unsigned)autoClimb.enabled,(unsigned)autoClimb.state,(unsigned)autoClimb.policy,
                       autoClimb.blend,autoClimb.trackingError,autoClimb.pitchBias,autoClimb.followingError,
                       (unsigned)autoClimb.flags,climbIntegral,posM-climbXRef,climbDriveRef,
                       fallPrevention.hazardRisk,climbRecovery,autoClimb.stableSeconds,
                       (unsigned)autoClimb.kind,autoClimb.exitPending?autoClimb.probeSeconds:autoClimb.flatSeconds,
                       (unsigned)autoClimb.exitBlock,autoClimb.cooldownSeconds,
                       descentGuard.limit,descentGuard.excess,descentGuard.scale,(unsigned)descentGuard.active,
                       descentGuard.goal,autoClimb.overspeed);
        loopMaxUs = 0;
    }

    static uint32_t lastHubMs=0;
    if (nowMs-lastHubMs>=100 && Serial.availableForWrite()>=256) {
        lastHubMs=nowMs;
        const auto& l=hub.left().fb(); const auto& r=hub.right().fb();
        // HM keeps raw motor feedback; current commands are robot-forward.
        Serial.printf("HM,%lu,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%u,%u,%u,%u,%u\n",
            (unsigned long)nowMs,hub.currentCmdL(),hub.currentCmdR(),l.currentA,r.currentA,
            l.rpm,r.rpm,l.tempC,r.tempC,l.fault,r.fault,(unsigned)hub.saturated(),
            (unsigned)hub.linkOk(HUB_LINK_TIMEOUT_MS),(unsigned)hub.modeOk());
    }
}
