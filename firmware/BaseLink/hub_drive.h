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
    bool modeOk() const { return leftWheel.modeOk && rightWheel.modeOk && leftPort.fb().posValid && rightPort.fb().posValid; }
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
    uint8_t wantedMode() const { return P.drvMode > .5f ? m0601::MODE_CURRENT : m0601::MODE_VELOCITY; }
    M0601Port rightPort, leftPort;
    HubWheel rightWheel, leftWheel;
    hub_control::CurrentLoop controller;
    bool enabled = false, currentLimited = false;
    float balanceAcceleration = 0;
    bool balanceControl = false, recoveryControl = false;
    uint32_t lastDriveUs = 0, lastQueryMs = 0;
};
extern HubDrive hub;
