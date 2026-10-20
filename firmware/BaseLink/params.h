/*
 * ============================================================
 *  params.h — every dynamics-altering value, live-tunable
 * ============================================================
 *  config.h stays the source of DEFAULTS. This file makes a RAM
 *  copy of each tunable, adds min/max bounds, NVS persistence and
 *  a serial protocol so the Radxa GUI can read and write them.
 *
 *  Control code reads P.<field> instead of the macro. Nothing
 *  about the control LAW changed — only where the numbers live.
 *
 *  DERIVED values (stepsPerM, countsPerM, vMax, aMax) are
 *  recomputed by params_recompute() whenever anything they depend
 *  on changes, so setting wheelR fixes the whole chain at once.
 *
 *  SAFETY: bounds are enforced on every write, from serial or
 *  anywhere else. TORCH_MAX_PCT stays a COMPILE-TIME hard cap in
 *  payload.h and is deliberately NOT exposed here.
 *
 *  PROTOCOL (see PROTOCOL.md):
 *    P?            dump every parameter, then "P."
 *    P,<key>,<val> set one (bounds-checked, replies P! or PE)
 *    PS            save all to NVS       PL  reload from NVS
 *    PD            restore compiled defaults (does NOT auto-save)
 * ============================================================
 */

#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// ============================================================
//  THE TABLE.  X(field, key, default, min, max, group, description)
//  key must be <= 11 chars and unique. group drives the GUI tabs.
// ============================================================
#define PARAM_TABLE(X)                                                                        \
  /* ---------- BALANCE: the LQR/LQI gain vector ---------- */                                \
  X(k1,        "K1",      LQR_K1,            -40.0f,   0.0f, "BALANCE", "position gain (per m)")        \
  X(k2,        "K2",      LQR_K2,            -40.0f,   0.0f, "BALANCE", "velocity gain (per m/s)")      \
  X(k3,        "K3",      LQR_K3,           -120.0f,   0.0f, "BALANCE", "pitch gain (per rad)")         \
  X(k4,        "K4",      LQR_K4,            -20.0f,   0.0f, "BALANCE", "pitch-rate damping (per rad/s)")\
  X(k5,        "K5",      LQR_K5,            -30.0f,   0.0f, "BALANCE", "position integral (per m*s)")  \
  X(pitchTrim, "TRIM",    PITCH_TRIM_DEG,     -8.0f,   8.0f, "BALANCE", "static pitch trim (deg)")      \
  X(zIntLim,   "ZLIM",    Z_INT_LIM,           0.05f, 10.0f, "BALANCE", "integral anti-windup (m*s)")   \
  X(exClamp,   "EXCLAMP", EX_CLAMP_M,          0.02f,  1.0f, "BALANCE", "max position error fed to LQR (m)") \
  X(vsatBleed, "VSATBLD", VSAT_BLEED,          0.0f,  20.0f, "BALANCE", "hold-point bleed when saturated (1/s)") \
  X(brakeLook, "BRAKELK", BRAKE_LOOKAHEAD_S,   0.0f,   2.0f, "BALANCE", "hold-point lead while driving (s)") \
  /* ---------- STIFF HOLD: second gain set ---------- */                                     \
  X(s1,        "S1",      LQR_S1,            -80.0f,   0.0f, "BALANCE", "stiff: position gain")         \
  X(s2,        "S2",      LQR_S2,            -60.0f,   0.0f, "BALANCE", "stiff: velocity gain")         \
  X(s3,        "S3",      LQR_S3,           -140.0f,   0.0f, "BALANCE", "stiff: pitch gain")            \
  X(s4,        "S4",      LQR_S4,            -25.0f,   0.0f, "BALANCE", "stiff: pitch-rate gain")       \
  X(s5,        "S5",      LQR_S5,            -40.0f,   0.0f, "BALANCE", "stiff: integral gain")         \
  /* ---------- BLDC MOTOR / GEOMETRY ---------- */                                           \
  X(wheelR,    "WHEELR",  WHEEL_RADIUS_M,      0.020f, 0.120f,"MOTOR", "loaded wheel radius (m)")       \
  X(maxSpdSt,  "MAXSPD",  MAX_SPEED_STEPS,     0.0f,25000.0f,"MOTOR", "counts/s; 0 = no firmware speed cap") \
  X(motAccel,  "MAXACC",  MOTOR_ACCEL_LIMIT,1000.0f,150000.0f,"MOTOR","balance acceleration (counts/s^2)") \
  X(motorMa,   "CURRENT", MOTOR_CURRENT_MA,  300.0f,MOTOR_CURRENT_MAX_MA,"MOTOR","per-motor torque current limit (mA)") \
  X(supplyV,   "SUPPLYV", SUPPLY_V,            7.0f, 24.0f,"MOTOR", "configured supply; speed estimate only (V)") \
  X(freeRpm,   "FREERPM", 0.0f,               0.0f,500.0f,"MOTOR", "measured free speed; 0 = voltage estimate (rpm)") \
  X(spdHead,   "SPDHEAD", SPEED_HEADROOM,      0.30f, 1.0f,"MOTOR", "full-stick share of free speed kept for balance") \
  X(drvMode,   "DRVMODE", HUB_DRIVE_MODE,      0.0f,  1.0f,"MOTOR", "1 current balance; 0 bench velocity") \
  X(hubKp,     "HUBKP",   HUB_KP,              0.0f, 40.0f,"MOTOR", "common velocity P (A per m/s)") \
  X(hubKi,     "HUBKI",   HUB_KI,              0.0f,100.0f,"MOTOR", "common velocity I (A per m)")   \
  X(hubIntLim, "HUBILIM", HUB_INT_LIM,         0.0f,  3.0f,"MOTOR", "common integral limit (A)")   \
  X(hubKff,    "HUBKFF",  HUB_KFF,             0.0f,  2.0f,"MOTOR", "common acceleration feed-forward") \
  X(hubMass,   "HUBMASS", HUB_MASS_KG,         0.3f, 10.0f,"MOTOR", "robot mass for feed-forward (kg)") \
  X(hubVSrc,   "HUBVSRC", HUB_VEL_SOURCE,      0.0f,  1.0f,"MOTOR", "0 corrected rpm; 1 position velocity") \
  X(hubAcc,    "HUBACC",  HUB_VEL_ACCEL_BYTE,  0.0f,255.0f,"MOTOR", "bench velocity ramp byte")    \
  X(dirL,      "LDIR",    LEFT_MOTOR_DIR,     -1.0f,  1.0f,"MOTOR", "left direction (+1/-1)")      \
  X(dirR,      "RDIR",    RIGHT_MOTOR_DIR,    -1.0f,  1.0f,"MOTOR", "right direction (+1/-1)")     \
  X(curSign,   "ISGN",    HUB_CURRENT_SIGN,   -1.0f, -1.0f,"MOTOR", "confirmed current polarity (-1)") \
  X(rpmSign,   "RPMSGN",  HUB_RPM_SIGN,       -1.0f, -1.0f,"MOTOR", "confirmed rpm polarity (-1)") \
  X(hubYawKp,  "HUBYAWKP",HUB_YAW_KP,         0.0f, 10.0f,"MOTOR", "differential velocity P (A per m/s)") \
  X(hubYawKi,  "HUBYAWKI",HUB_YAW_KI,         0.0f, 30.0f,"MOTOR", "differential velocity I (A per m)") \
  X(hubYawLim, "HUBYAWIL",HUB_YAW_INT_LIM,    0.0f,  1.0f,"MOTOR", "turn integral limit (A)")     \
  X(hubYawSlew,"HUBYAWSL",HUB_YAW_CURRENT_SLEW,0.5f,30.0f,"MOTOR", "turn current slew (A/s)")     \
  /* ---------- DRIVE INPUT ---------- */                                                     \
  X(maxDriveV, "DRIVEV",  MAX_DRIVE_VEL_MS,    0.05f,  3.0f, "DRIVE",   "full-stick speed (m/s)")       \
  X(driveDb,   "DRIVEDB", DRIVE_DEADBAND,      0.0f,   0.3f, "DRIVE",   "stick deadband -> position hold")\
  X(spdLoDrv,  "LOWDRV",  SPEED_LO_DRIVE_SCALE,0.05f,  1.0f, "DRIVE",   "LOW mode drive scale")         \
  X(spdLoStr,  "LOWSTR",  SPEED_LO_STEER_SCALE,0.05f,  1.0f, "DRIVE",   "LOW mode steer scale")         \
  X(joyFwdSc,  "JOYFWD",  JOY_FWD_SCALE,      -2.0f,   2.0f, "DRIVE",   "pad fwd units -> normalised")  \
  X(joyStrSc,  "JOYSTR",  JOY_STEER_SCALE,    -2.0f,   2.0f, "DRIVE",   "pad steer scale")              \
  /* ---------- STICK SHAPING ---------- */                                                   \
  X(stickDz,   "STICKDZ", STICK_DEADZONE,      0.0f,   0.4f, "DRIVE",   "radial stick deadzone")        \
  X(snapIn,    "SNAPIN",  SNAP_IN_DEG,         0.0f,  40.0f, "DRIVE",   "zero-steer cone (deg)")        \
  X(snapOut,   "SNAPOUT", SNAP_OUT_DEG,        1.0f,  70.0f, "DRIVE",   "full-steer angle (deg)")       \
  X(expoSteer, "EXPOSTR", EXPO_STEER,          0.0f,   1.0f, "DRIVE",   "steer expo (0 lin, 1 cubic)")  \
  X(expoDrive, "EXPODRV", EXPO_DRIVE,          0.0f,   1.0f, "DRIVE",   "drive expo")                   \
  /* ---------- YAW / STEERING ---------- */                                                  \
  X(maxSteerSt,"MAXSTR",  MAX_STEER_STEPS,   200.0f, 9000.0f,"YAW",     "max differential (counts/s)")  \
  X(steerSlew, "STRSLEW", STEER_SLEW,       1000.0f,40000.0f,"YAW",     "steer slew (counts/s^2)")      \
  X(steerDb,   "STRDB",   STEER_DEADBAND,      0.0f,   0.3f, "YAW",     "steer deadband")               \
  X(yawKp,     "YAWKP",   YAW_HOLD_KP,         0.0f,  20.0f, "YAW",     "heading hold P (steps/s per count)")\
  X(yawKd,     "YAWKD",   YAW_HOLD_KD,         0.0f,   1.0f, "YAW",     "heading hold D")               \
  X(yawMaxSt,  "YAWMAX",  YAW_HOLD_MAX_STEPS,  0.0f, 6000.0f,"YAW",     "heading hold authority cap")   \
  X(yawRelatch,"YAWRELCH",YAW_RELATCH_COUNTS,100.0f,20000.0f,"YAW",     "slip re-latch threshold (counts)")\
  X(yawTiltSus,"YAWTILT", YAW_TILT_SUSPEND_DEG,0.0f,  45.0f, "YAW",     "suspend heading hold above (deg)")\
  X(trackW,    "TRACKW",  TRACK_WIDTH_M,       0.05f,  0.60f,"YAW",     "wheel-to-wheel (m)")           \
  X(lookMaxDeg,"LOOKMAX", LOOK_MAX_DEG,        0.0f,  60.0f, "YAW",     "LOOK gesture yaw offset (deg)")\
  /* ---------- SAFETY ---------- */                                                          \
  X(armAngle,  "ARMANG",  ARM_ANGLE_DEG,       1.0f,  20.0f, "SAFETY",  "must be this upright to arm (deg)")\
  X(maxTilt,   "MAXTILT", MAX_TILT_ANGLE,     10.0f,  85.0f, "SAFETY",  "fall cutoff (deg)")            \
  X(maxRate,   "MAXRATE", MAX_PITCH_RATE_SAFETY,50.0f,2000.0f,"SAFETY", "rate-spike cutoff (deg/s)")    \
  X(radxaTmo,  "RADXATMO",RADXA_TIMEOUT_MS,   50.0f, 5000.0f,"SAFETY",  "Radxa link freshness (ms)")    \
  X(joyTmo,    "JOYTMO",  JOY_TIMEOUT_MS_CFG,  50.0f,5000.0f,"SAFETY",  "pad link freshness (ms)")      \
  X(bootWait,  "BOOTWAIT",HOST_BOOT_WAIT,      0.0f,   1.0f,"SAFETY",  "wait for Radxa ACK before startup/arming") \
  /* ---------- IMU / SIGNS ---------- */                                                     \
  X(imuCutoff, "IMUCUT",  IMU_FILTER_CUTOFF_HZ,2.0f, 100.0f, "IMU",     "pitch filter cutoff (Hz)")     \
  X(mahonyKp,  "MAHKP",   MAHONY_KP,           0.0f,  20.0f, "IMU",     "Mahony Kp")                    \
  X(mahonyKi,  "MAHKI",   MAHONY_KI,           0.0f,   1.0f, "IMU",     "Mahony Ki")                    \
  X(pitchSign, "PITCHSGN",PITCH_FWD_SIGN,     -1.0f,   1.0f, "IMU",     "pitch sign (+1/-1)")           \
  X(encSign,   "ENCSGN",  ENC_FWD_SIGN,        1.0f,   1.0f, "IMU",     "hub forward frame (fixed +1)") \
  X(rateSign,  "RATESGN", RATE_FWD_SIGN,      -1.0f,   1.0f, "IMU",     "gyro-rate sign (+1/-1)")       \
  /* ---------- CLIMB MODE ---------- */                                                      \
  X(climbZLim, "CLMBZ",   CLIMB_Z_INT_LIM,     0.1f,  10.0f, "CLIMB",   "climb integral clamp (m*s)")   \
  X(climbVel,  "CLMBV",   CLIMB_VEL_MS,        0.02f,  1.5f, "CLIMB",   "climb reference ramp (m/s)")   \
  X(climbStr,  "CLMBSTR", CLIMB_STEER_SCALE,   0.0f,   1.0f, "CLIMB",   "steer authority while climbing")\
  /* ---------- LED RING ---------- */                                                        \
  X(ledBright, "LEDBRI",  LED_MAX_BRIGHT,      0.0f, 255.0f, "LED",     "brightness cap (0-255)")       \
  X(ledFront,  "LEDFRONT",LED_FRONT_INDEX,     0.0f,  63.0f, "LED",     "which LED faces forward")      \
  /* ---------- PAYLOAD (torch duty stays hard-capped in payload.h) ---------- */             \
  X(servoYawR, "SVYAWR",  SERVO_YAW_RATE_DPS,  5.0f, 300.0f, "PAYLOAD", "pan rate (deg/s)")             \
  X(servoZoomR,"SVZOOMR", SERVO_ZOOM_RATE_DPS, 5.0f, 300.0f, "PAYLOAD", "zoom rate (deg/s)")            \
  X(svYawMin,  "SVYAWMIN",SERVO_YAW_MIN_DEG,   0.0f, 270.0f, "PAYLOAD", "pan min (deg)")                \
  X(svYawMax,  "SVYAWMAX",SERVO_YAW_MAX_DEG,   0.0f, 270.0f, "PAYLOAD", "pan max (deg)")                \
  X(svZoomMin, "SVZMIN",  SERVO_ZOOM_MIN_DEG,  0.0f, 270.0f, "PAYLOAD", "zoom min (deg)")               \
  X(svZoomMax, "SVZMAX",  SERVO_ZOOM_MAX_DEG,  0.0f, 270.0f, "PAYLOAD", "zoom max (deg)")               \
  X(torchStep, "TRCHSTP", TORCH_STEP_PCT,      1.0f,  25.0f, "PAYLOAD", "torch dim step (%)") \
  /* ---------- CLIMB gain set (was compile-time only) ---------- */         \
  X(c1,        "C1",      LQR_C1,            -60.0f,   0.0f, "CLIMB",   "climb: position gain")         \
  X(c2,        "C2",      LQR_C2,            -50.0f,   0.0f, "CLIMB",   "climb: velocity gain")         \
  X(c3,        "C3",      LQR_C3,           -140.0f,   0.0f, "CLIMB",   "climb: pitch gain")            \
  X(c4,        "C4",      LQR_C4,            -25.0f,   0.0f, "CLIMB",   "climb: pitch-rate gain")       \
  X(c5,        "C5",      LQR_C5,            -30.0f,   0.0f, "CLIMB",   "climb: integral gain")         \
  /* ---------- magnetometer calibration ---------- */                       \
  X(magOffX,   "MAGOFFX", MAG_OFFSET_X,    -3000.0f,3000.0f, "IMU",     "hard-iron offset X")           \
  X(magOffY,   "MAGOFFY", MAG_OFFSET_Y,    -3000.0f,3000.0f, "IMU",     "hard-iron offset Y")           \
  X(magOffZ,   "MAGOFFZ", MAG_OFFSET_Z,    -3000.0f,3000.0f, "IMU",     "hard-iron offset Z")           \
  X(magSignX,  "MAGSGNX", MAG_SIGN_X,         -1.0f,   1.0f, "IMU",     "mag axis sign X")              \
  X(magSignY,  "MAGSGNY", MAG_SIGN_Y,         -1.0f,   1.0f, "IMU",     "mag axis sign Y")              \
  X(magSignZ,  "MAGSGNZ", MAG_SIGN_Z,         -1.0f,   1.0f, "IMU",     "mag axis sign Z")              \
  X(accelSign, "ACCSGN",  PITCH_ACCEL_SIGN,   -1.0f,   1.0f, "IMU",     "accel tilt sign")              \
  X(gyroSign,  "GYROSGN", PITCH_GYRO_SIGN,    -1.0f,   1.0f, "IMU",     "gyro axis sign")               \
  /* ---------- gamepad mapping ---------- */                                \
  X(padDz,     "PADDZ",   PAD_DEADZONE,        0.0f,   0.4f, "DRIVE",   "legacy per-axis deadzone")     \
  X(padFwdSgn, "PADFWD",  PAD_FWD_SIGN,       -1.0f,   1.0f, "DRIVE",   "drive axis sign")              \
  X(padStrSgn, "PADSTR",  PAD_STEER_SIGN,     -1.0f,   1.0f, "DRIVE",   "steer axis sign")              \
  X(padRpt1,   "PADRPT1", PAD_REPEAT_FIRST_MS, 50.0f,2000.0f,"DRIVE",   "d-pad repeat delay (ms)")      \
  X(padRpt,    "PADRPT",  PAD_REPEAT_MS,      20.0f, 1000.0f,"DRIVE",   "d-pad repeat rate (ms)")       \
  X(bootHigh,  "BOOTHIGH",SPEED_BOOT_HIGH,     0.0f,   1.0f, "DRIVE",   "boot in HIGH speed mode")      \
  /* ---------- payload detail ---------- */                                 \
  X(padYawSgn, "PADYAW",  PAD_YAW_SIGN,       -1.0f,   1.0f, "PAYLOAD", "pan axis sign")                \
  X(padZoomSgn,"PADZOOM", PAD_ZOOM_SIGN,      -1.0f,   1.0f, "PAYLOAD", "zoom axis sign")               \
  X(servoUsMin,"SVUSMIN", SERVO_US_MIN,      400.0f,1500.0f, "PAYLOAD", "servo min pulse (us)")         \
  X(servoUsMax,"SVUSMAX", SERVO_US_MAX,     1500.0f,2800.0f, "PAYLOAD", "servo max pulse (us)")         \
  X(svYawHome, "SVYAWHOME",SERVO_YAW_HOME_DEG, 0.0f, 270.0f, "PAYLOAD", "pan home (deg)")               \
  X(svZoomHome,"SVZHOME", SERVO_ZOOM_HOME_DEG, 0.0f, 270.0f, "PAYLOAD", "zoom home (deg)")              \
  X(svRate,    "SVRATE",  SERVO_UPDATE_HZ,    10.0f, 200.0f, "PAYLOAD", "servo refresh (Hz)")           \
  X(svIdleMs,  "SVIDLE",  SERVO_IDLE_RELEASE_MS,0.0f,10000.0f,"PAYLOAD","stop pulsing after idle (ms)") \
  X(torchBoot, "TRCHBOOT",TORCH_BOOT_PCT,      1.0f,  90.0f, "PAYLOAD", "torch preset at boot (%)")     \
  X(torchMin,  "TRCHMIN", TORCH_MIN_PCT,       1.0f,  50.0f, "PAYLOAD", "torch minimum (%)")            \
  /* ---------- ring ---------- */                                           \
  X(ledFps,    "LEDFPS",  LED_FPS,             5.0f, 120.0f, "LED",     "ring frame rate") \
  /* ---------- TERRAIN: rough ground / step descent ---------- */           \
  X(terrEnable,"TERREN",  1.0f,                0.0f,   1.0f, "TERRAIN", "adaptive terrain gains on/off")\
  X(terrThr,   "TERRTHR", 0.18f,               0.02f,  1.0f, "TERRAIN", "roughness for full softening")\
  X(terrSoft,  "TERRSOFT",0.55f,               0.25f,  1.0f, "TERRAIN", "K3/K4 multiplier when rough")\
  X(terrTau,   "TERRTAU", 0.35f,               0.02f,  3.0f, "TERRAIN", "roughness filter time const (s)")\
  X(dropG,     "DROPG",   0.55f,               0.10f,  0.95f,"TERRAIN", "free-fall threshold (g)")   \
  X(dropMs,    "DROPMS",  35.0f,               5.0f,  300.0f,"TERRAIN", "airborne confirm time (ms)")\
  X(landMs,    "LANDMS",  350.0f,             50.0f, 2000.0f,"TERRAIN", "soft-gain hold after landing (ms)")\
  X(airFreeze, "AIRFRZ",  1.0f,                0.0f,   1.0f, "TERRAIN", "freeze integrator while airborne")

// ============================================================
//  Struct, defaults, metadata
// ============================================================
struct Params {
#define X(f, k, d, lo, hi, g, desc)  float f;
    PARAM_TABLE(X)
#undef X
    // ---- derived, never set directly ----
    float stepsPerM, countsPerM, vMax, aMax;
    float vNoLoad, maxSpdEff;
    float driveVEff;  // m/s actually used for full stick: min(DRIVEV, SPDHEAD*free speed, MAXSPD)
};

struct ParamMeta {
    const char* key;
    size_t      offset;
    float       def, lo, hi;
    const char* group;
    const char* desc;
};

// ONE instance, defined in params.cpp. A "static" here would give every
// .cpp its own private copy — a genuinely horrible bug to chase.
extern Params            P;
extern const ParamMeta   PARAM_META[];
extern const size_t      PARAM_COUNT;

// Implemented by the sketch: pushes changed values into hardware
// (driver current, IMU filter, LED brightness...). See BaseLink.ino.
void params_applyHardware(bool currentChanged);
bool params_motionActive();

void   params_recompute();
float* params_ptr(size_t i);
int    params_index(const char* key);
int    params_set(const char* key, float v, bool quiet = false);
void   params_defaults();
void   params_save();
void   params_load();
void   params_dump();
void   params_service();
bool   params_handleLine(char* line);
void   params_begin();

#endif // PARAMS_H
