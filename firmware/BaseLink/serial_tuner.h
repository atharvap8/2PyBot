/*
 * ============================================================
 *  serial_tuner.h — Live PID Tuning via Serial Console
 * ============================================================
 *  Non-blocking text command interpreter for adjusting PID
 *  gains, motor state, and IMU calibration at runtime without
 *  reflashing.
 *
 *  Commands:
 *  ─────────────────────────────────────────────────
 *    P=<val>   Set Kp
 *    I=<val>   Set Ki
 *    D=<val>   Set Kd
 *    T=<val>   Set balance setpoint (°)
 *    M=<val>   Set motor RMS current limit (mA)
 *    S         Print current parameters
 *    R         Reset PID integrator and history
 *    E         Enable motors
 *    X         Emergency stop
 *    C         Recalibrate gyro bias
 *    ?         Print command reference
 * ============================================================
 */

#ifndef SERIAL_TUNER_H
#define SERIAL_TUNER_H

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"
#include "stepper_control.h"
#include "imu_sensor.h"

extern bool DEBUG;

class SerialTuner {
public:
    // Takes references to the live PID, stepper, and IMU objects.
    SerialTuner(PIDController& pid, StepperControl& stp, IMUSensor& imu)
        : _pid(pid), _stp(stp), _imu(imu) {}

    void begin() {
        printHelp();
    }

    // Call once per main loop iteration. Reads one line from Serial
    // and dispatches the appropriate action. Returns true if a command
    // was processed.
    bool process() {
        if (!Serial.available()) return false;

        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return false;

        char cmd = toupper(line.charAt(0));
        float val = 0;

        int eqIdx = line.indexOf('=');
        if (eqIdx > 0) {
            val = line.substring(eqIdx + 1).toFloat();
        }

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
                Serial.println("[TUNE] 'A' command deprecated — Mahony filter active.");
                break;
            case 'M':
                if (eqIdx > 0) { _stp.setCurrent((uint16_t)val); }
                break;
            case 'S':
                printSettings();
                break;
            case 'R':
                _pid.reset();
                Serial.println("[TUNE] PID state cleared.");
                break;
            case 'E':
                _pid.reset();
                _stp.enable();
                break;
            case 'X':
                _stp.setSpeed(0);
                _stp.disable();
                Serial.println("[TUNE] *** EMERGENCY STOP ***");
                break;
            case 'C':
                _stp.setSpeed(0);
                _stp.disable();
                _imu.calibrateGyro();
                Serial.println("[TUNE] Press 'E' to re-enable.");
                break;
            case '?':
                printHelp();
                break;
            case 'L':
                DEBUG = !DEBUG;
                break;
            default:
                Serial.printf("[TUNE] Unknown command: %s\n", line.c_str());
                break;
        }
        return true;
    }

private:
    PIDController&  _pid;
    StepperControl& _stp;
    IMUSensor&      _imu;

    void printSettings() {
        Serial.println("\n=======================================");
        Serial.println("         CURRENT PID SETTINGS          ");
        Serial.println("=======================================");
        Serial.printf ("    Kp       = %10.4f\n", _pid.Kp);
        Serial.printf ("    Ki       = %10.4f\n", _pid.Ki);
        Serial.printf ("    Kd       = %10.4f\n", _pid.Kd);
        Serial.printf ("    Target   = %10.2f °\n", _pid.setpoint);
        Serial.printf ("    Motors   = %s\n", _stp.isEnabled() ? "ENABLED" : "DISABLED");
        Serial.println("=======================================\n");
    }

    void printHelp() {
        Serial.println("\n---- Self-Balancing Robot — Serial Tuner ----");
        Serial.println("  P=<val>   Set Kp             I=<val>   Set Ki");
        Serial.println("  D=<val>   Set Kd             T=<val>   Set Setpoint (°)");
        Serial.println("  M=<val>   Set current (mA)   S         Print settings");
        Serial.println("  R         Reset PID           E         Enable motors");
        Serial.println("  X         Emergency stop      C         Calibrate gyro");
        Serial.println("  ?         Print this help");
        Serial.println("---------------------------------------------\n");
    }
};

#endif // SERIAL_TUNER_H
