/*
 * ============================================================
 *  serial_tuner.h — Runtime Live PID Tuning 
 * ============================================================
 *  Supplies an asynchronous, non-blocking text interpretation 
 *  interface allowing immediate structural adjustment of PID 
 *  coefficients, hardware parameters, and motor states directly
 *  through the live Bluetooth or USB Serial console.
 *
 *  Utilizing this interface entirely avoids the debilitating 
 *  compile-and-flash cycle traditionally plaguing robotic 
 *  calibration procedures.
 *
 *  Commands:
 *  ─────────────────────────────────────────────────
 *    P=12.5      Assign Proportional element multiplier (Kp).
 *    I=0.3       Assign Integral element multiplier (Ki).
 *    D=1.0       Assign Derivative element multiplier (Kd).
 *    T=2.5       Assign exact angular coordinate setpoint (°).
 *    M=600       Assign strict motor RMS amperage limits (mA).
 *    S           Render existing parameter settings logically.
 *    R           Re-initialize the PID calculus, clearing errors.
 *    E           Assert hardware enable pins, resuming locomotion.
 *    X           Invoke emergency motor disconnection instantly.
 *    C           Re-execute precise gyroscopic zero-bias calibration.
 *    ?           Render the command syntax dictionary.
 * ============================================================
 */

#ifndef SERIAL_TUNER_H
#define SERIAL_TUNER_H

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"
#include "stepper_control.h"
#include "imu_sensor.h"

// ============================================================
//  Global Configuration Hook
// ============================================================
extern bool DEBUG;

class SerialTuner {
public:
    // ============================================================
    //  Constructor
    // ============================================================
    // Accepts memory references natively mapped directly into the 
    // fundamental functional engines.
    SerialTuner(PIDController& pid, StepperControl& stp, IMUSensor& imu)
        : _pid(pid), _stp(stp), _imu(imu) {}

    // ============================================================
    //  Initialization
    // ============================================================
    void begin() {
        printHelp();
    }

    // ============================================================
    //  Processor execution cycle
    // ============================================================
    // Must be invoked consistently at the baseline loop limit.
    // Detects pending payload buffers and safely extracts alphanumeric patterns.
    bool process() {
        // Abandon operations if identical buffers are empty.
        if (!Serial.available()) return false;

        // Extract native serial boundaries cautiously.
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return false;

        // Standardize syntax extraction protocols.
        char cmd = toupper(line.charAt(0));
        float val = 0;

        // Interpret equivalent boundary directives intelligently.
        int eqIdx = line.indexOf('=');
        if (eqIdx > 0) {
            val = line.substring(eqIdx + 1).toFloat();
        }

        // ============================================================
        //  Command Switch Execution
        // ============================================================
        switch (cmd) {
            case 'P':
                if (eqIdx > 0) { _pid.Kp = val; Serial.printf("[TUNE] Kp = %.4f\n", val); }
                break;
            case 'I':
                if (eqIdx > 0) { _pid.Ki = val; Serial.printf("[TUNE] Ki = %.4f\n", val); }
                break;
            case 'D':
                if (eqIdx > 0) { _pid.Kd = val; Serial.printf("[TUNE] Kd = %.4f\n", val); }
                break;
            case 'T':
                if (eqIdx > 0) { _pid.setpoint = val; Serial.printf("[TUNE] Target = %.2f°\n", val); }
                break;
            case 'A':
                Serial.println("[TUNE] Alpha setting identically deprecated (Mahony algorithms active).");
                break;
            case 'M':
                if (eqIdx > 0) {
                    _stp.setCurrent((uint16_t)val);
                }
                break;
            case 'S':
                printSettings();
                break;
            case 'R':
                _pid.reset();
                Serial.println("[TUNE] PID memory cache entirely reset.");
                break;
            case 'E':
                _pid.reset();
                _stp.enable();
                break;
            case 'X':
                _stp.setSpeed(0);
                _stp.disable();
                Serial.println("[TUNE] *** CRITICAL EMERGENCY STOP INVOKED ***");
                break;
            case 'C':
                _stp.setSpeed(0);
                _stp.disable();
                _imu.calibrateGyro();
                Serial.println("[TUNE] Re-enable hardware interfaces pressing 'E' whenever ready.");
                break;
            case '?':
                printHelp();
                break;
            case 'L':
                DEBUG = !DEBUG;
                break;
            default:
                Serial.printf("[TUNE] Syntax evaluation error unknown command: %s\n", line.c_str());
                break;
        }
        return true;
    }

private:
    PIDController&  _pid;
    StepperControl& _stp;
    IMUSensor&      _imu;

    // ============================================================
    //  Formatting Helpers
    // ============================================================
    void printSettings() {
        Serial.println("\n=======================================");
        Serial.println("         CURRENT PID SETTINGS          ");
        Serial.println("=======================================");
        Serial.printf ("    Kp       = %10.4f            \n", _pid.Kp);
        Serial.printf ("    Ki       = %10.4f            \n", _pid.Ki);
        Serial.printf ("    Kd       = %10.4f            \n", _pid.Kd);
        Serial.printf ("    Target   = %10.2f °          \n", _pid.setpoint);
        Serial.printf ("    Motors   = %s               \n",
                          _stp.isEnabled() ? "ENABLED " : "DISABLED");
        Serial.println("=======================================\n");
    }

    void printHelp() {
        Serial.println("\n---- Self-Balancing Robot — Serial Tuner Console ----");
        Serial.println("  P=<val>   Define Kp               I=<val>   Define Ki");
        Serial.println("  D=<val>   Define Kd               T=<val>   Define Setpoint (°)");
        Serial.println("  M=<val>   Define Motor limit (mA) S         Output State Config");
        Serial.println("  R         Reset Process Arrays    E         Enable Hardware");
        Serial.println("  X         Catastrophic Terminate  C         Calibrate Metrics");
        Serial.println("  ?         Render syntax logic                        ");
        Serial.println("----------------------------------------------------\n");
    }
};

#endif // SERIAL_TUNER_H
