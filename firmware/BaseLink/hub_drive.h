#pragma once
#include "config.h"
#include "params.h"
#include "m0601.h"
#include "hub_control.h"

struct HubWheel {
    explicit HubWheel(M0601Port* p) : port(p) {}
    M0601Port* port;
    float direction = 1, target = 0, velocity = 0, current = 0;
    int64_t position = 0;
    uint16_t previousPosition = 0;
    uint32_t positionFrames = 0, lastPositionUs = 0;
    bool initialized = false, modeOk = false, queryNext = false;
    uint8_t lastFault = 0;
};

class HubDrive {
public:
    HubDrive();
    bool begin();
    void service(float dt);
    void transmit(float dt);
    void setSpeeds(float leftCps, float rightCps);
    void setBalanceCommand(float leftCps, float rightCps, float acceleration, bool recovering);
    void setSpeed(float cps) { setSpeeds(cps, cps); }
    void enable();
    void disable();
    bool isEnabled() const { return enabled; }
    int64_t getPositionL() const;
    int64_t getPositionR() const;
    float wheelVelL() const { return measuredVelocity(leftWheel); }
    float wheelVelR() const { return measuredVelocity(rightWheel); }
    bool modeOk() const { return confirmedMode(leftWheel) && confirmedMode(rightWheel) && leftPort.fb().posValid && rightPort.fb().posValid; }
    bool wheelHealthy(const HubWheel& w) const { return confirmedMode(w) && w.port->fb().posValid && !w.port->fb().fault; }
    bool balanceModeSelected() const { return P.drvMode == 1 || P.drvMode == 2; }
    bool velocityMode() const { return wantedMode() == m0601::MODE_VELOCITY; }
    float velocityLimit() const {
        const float protocol = m0601::RPM_CMD_MAX * (2.0f * PI * P.wheelR / 60.0f);
        return P.vMax > 0 ? fminf(P.vMax, protocol) : protocol;
    }
    int16_t velocityCmdL() const { return rpmCommandL; }
    int16_t velocityCmdR() const { return rpmCommandR; }
    bool linkOk(uint32_t ageMs) const;
    uint8_t faultBits() const { return leftPort.fb().fault | rightPort.fb().fault; }
    const M0601Port& left() const { return leftPort; }
    const M0601Port& right() const { return rightPort; }
    float currentCmdL() const { return leftWheel.current; }
    float currentCmdR() const { return rightWheel.current; }
    bool saturated() const { return currentLimited; }
    void applyTuning();
    void printStatus() const;

private:
    void serviceWheel(HubWheel& wheel);
    void handshake(HubWheel& wheel);
    float measuredVelocity(const HubWheel& wheel) const;
    int64_t counts(const HubWheel& wheel) const;
    uint8_t wantedMode() const { return P.drvMode == 1 ? m0601::MODE_CURRENT : m0601::MODE_VELOCITY; }
    bool confirmedMode(const HubWheel& w) const { return w.modeOk && w.port->fb().mode == wantedMode(); }
    M0601Port rightPort, leftPort;
    HubWheel rightWheel, leftWheel;
    hub_control::CurrentLoop controller;
    bool enabled = false, currentLimited = false;
    float balanceAcceleration = 0;
    bool balanceControl = false, recoveryControl = false;
    uint32_t lastDriveUs = 0, lastQueryMs = 0;
    uint8_t configuredMode = 0;
    int16_t rpmCommandL = 0, rpmCommandR = 0;
    void reportFaults(HubWheel& wheel);
};
extern HubDrive hub;
