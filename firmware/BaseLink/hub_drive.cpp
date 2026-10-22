#include "hub_drive.h"

HubDrive hub;
HubDrive::HubDrive()
    : rightPort(Serial2, RIGHT_MOTOR_ID, RIGHT_RS485_RX, RIGHT_RS485_TX, RIGHT_RS485_DE, "RIGHT"),
      leftPort(Serial1, LEFT_MOTOR_ID, LEFT_RS485_RX, LEFT_RS485_TX, LEFT_RS485_DE, "LEFT"),
      rightWheel{&rightPort}, leftWheel{&leftPort} {}

void HubDrive::applyTuning() {
    const uint8_t selected = wantedMode();
    if (configuredMode && configuredMode != selected) {
        disable();
        leftWheel.modeOk = rightWheel.modeOk = false;
        leftWheel.queryNext = rightWheel.queryNext = false;
    }
    configuredMode = selected;
    leftWheel.direction = P.dirL < 0 ? -1 : 1;
    rightWheel.direction = P.dirR < 0 ? -1 : 1;
}

bool HubDrive::begin() {
    applyTuning(); rightPort.begin(); leftPort.begin(); disable();
    uint32_t start = millis();
    while (millis() - start < 1500) {
        service(LOOP_PERIOD_US * 1e-6f);
        if (modeOk() && leftWheel.initialized && rightWheel.initialized) break;
        transmit(LOOP_PERIOD_US * 1e-6f);
        delay(5);
    }
    Serial.printf("[HUB] RIGHT id%u %s; LEFT id%u %s\n", rightPort.id(), rightWheel.modeOk ? "OK" : "NO REPLY",
                  leftPort.id(), leftWheel.modeOk ? "OK" : "NO REPLY");
    return modeOk();
}

void HubDrive::serviceWheel(HubWheel& w) {
    w.port->service();
    const auto& fb = w.port->fb();
    w.modeOk = fb.frames && fb.mode == wantedMode() && millis() - fb.rxMs < HUB_LINK_TIMEOUT_MS;
    if (fb.posFrames == w.positionFrames) return;
    const uint32_t now = micros();
    if (!w.initialized) { w.previousPosition = fb.pos16; w.initialized = true; w.lastPositionUs = now; }
    int32_t delta = (int32_t)fb.pos16 - w.previousPosition;
    if (delta > 16383) delta -= 32767;
    if (delta < -16383) delta += 32767;
    const float elapsed = (now - w.lastPositionUs) * 1e-6f;
    if (elapsed > 0) {
        const float raw = delta / m0601::POS_PER_REV * (2 * PI * P.wheelR) * w.direction / elapsed;
        w.velocity += (1 - expf(-2 * PI * 30 * elapsed)) * (raw - w.velocity);
    }
    w.position += delta; w.previousPosition = fb.pos16;
    w.positionFrames = fb.posFrames; w.lastPositionUs = now;
}

void HubDrive::service(float) {
    serviceWheel(leftWheel); serviceWheel(rightWheel);
    reportFaults(leftWheel); reportFaults(rightWheel);
}
float HubDrive::measuredVelocity(const HubWheel& w) const {
    return P.hubVSrc > .5f ? w.velocity : w.port->fb().rpm * P.rpmSign * w.direction * (2 * PI * P.wheelR / 60);
}
int64_t HubDrive::counts(const HubWheel& w) const {
    // No synthetic RPM bridge/reset: a query must never create an encoder jump.
    return llround(w.position * (double)w.direction * M0601_CPR / m0601::POS_PER_REV);
}
int64_t HubDrive::getPositionL() const { return counts(leftWheel); }
int64_t HubDrive::getPositionR() const { return counts(rightWheel); }
bool HubDrive::linkOk(uint32_t ageMs) const {
    return leftPort.fb().frames && rightPort.fb().frames && millis() - leftPort.fb().rxMs < ageMs &&
           millis() - rightPort.fb().rxMs < ageMs;
}
void HubDrive::setSpeeds(float leftCps, float rightCps) {
    leftWheel.target = leftCps / P.countsPerM; rightWheel.target = rightCps / P.countsPerM;
    balanceControl = recoveryControl = false;
}
void HubDrive::setBalanceCommand(float leftCps, float rightCps, float acceleration, bool recovering) {
    setSpeeds(leftCps, rightCps);
    balanceAcceleration = acceleration;
    balanceControl = true;
    recoveryControl = recovering;
}
void HubDrive::enable() {
    if (!enabled) {
        controller.reset(.5f * (leftWheel.target + rightWheel.target));
        lastDriveUs = 0; enabled = true;
    }
}
void HubDrive::disable() {
    enabled = false; currentLimited = false; controller.reset();
    leftWheel.target = rightWheel.target = leftWheel.current = rightWheel.current = 0;
    balanceControl = recoveryControl = false; balanceAcceleration = 0;
    rpmCommandL = rpmCommandR = 0;
    lastDriveUs = 0;
}

void HubDrive::handshake(HubWheel& w) {
    // Mode not (yet) confirmed: alternate MODE and QUERY. Neither moves the
    // wheel. Every reply, DRIVE or QUERY, reports the motor's actual mode.
    if (w.queryNext) w.port->sendQuery();
    else w.port->sendMode(wantedMode());
    w.queryNext = !w.queryNext;
}

void HubDrive::transmit(float dt) {
    // v4.0.3 transport: exactly one frame per motor per control cycle, never
    // gated on the previous reply (4.0.2 skipped drive frames while a reply
    // was outstanding and then resynchronised for >=10 ms, so one late or
    // lost reply froze that wheel's torque for ~25 ms and made the update
    // rate depend on loop jitter). Safety is unchanged: every reply carries
    // the motor's mode byte, so a motor that rebooted into velocity mode is
    // seen on the very next reply and the robot disarms.
    const bool healthy = wheelHealthy(leftWheel) && wheelHealthy(rightWheel);
    if (!healthy) {
        disable();
        for (HubWheel* w : {&leftWheel, &rightWheel}) {
            if (!confirmedMode(*w)) handshake(*w);
            else w->port->sendDriveRaw(0);   // confirmed mode, zero torque
        }
        return;
    }
    if (!enabled && millis() - lastQueryMs >= 500) {
        // Temperature only while disarmed: QUERY replies carry no pos16.
        leftPort.sendQuery(); rightPort.sendQuery(); lastQueryMs = millis(); return;
    }
    const uint32_t now = micros();
    if (lastDriveUs) dt = (now - lastDriveUs) * 1e-6f;
    lastDriveUs = now;
    if (wantedMode() == m0601::MODE_VELOCITY) {
        // Allocate common speed first; turning may use only the remaining
        // RPM range. Independent wheel clipping would erase catch speed.
        const float limit = velocityLimit();
        const float requestedCommon = .5f * (leftWheel.target + rightWheel.target);
        const float common = hub_control::clip(requestedCommon, -limit, limit);
        const float requestedTurn = .5f * (rightWheel.target - leftWheel.target);
        const float room = fmaxf(0, limit - fabsf(common));
        const float turn = hub_control::clip(requestedTurn, -room, room);
        const float scale = 60 / (2 * PI * P.wheelR);
        rpmCommandL = enabled ? lroundf(hub_control::clip((common-turn) * scale * leftWheel.direction * P.velSign, -m0601::RPM_CMD_MAX, m0601::RPM_CMD_MAX)) : 0;
        rpmCommandR = enabled ? lroundf(hub_control::clip((common+turn) * scale * rightWheel.direction * P.velSign, -m0601::RPM_CMD_MAX, m0601::RPM_CMD_MAX)) : 0;
        const uint8_t ramp = balanceControl ? 0 : (uint8_t)P.hubAcc;
        leftPort.sendDriveRaw(rpmCommandL, ramp);
        rightPort.sendDriveRaw(rpmCommandR, ramp);
        currentLimited = enabled && (fabsf(common-requestedCommon) > .00001f || fabsf(turn-requestedTurn) > .00001f);
        // The driver's velocity loop chooses current; these are not current
        // setpoints. Real current remains available in the HM feedback fields.
        leftWheel.current = rightWheel.current = 0;
        return;
    }
    rpmCommandL = rpmCommandR = 0;
    hub_control::Currents currents;
    if (enabled) {
        const hub_control::Settings settings{P.hubKp, P.hubKi, P.hubIntLim, P.hubKff, P.hubMass, P.wheelR, M0601_KT,
            P.hubYawKp, P.hubYawKi, P.hubYawLim, P.hubYawSlew, P.motorMa * .001f};
        currents = controller.update(leftWheel.target, rightWheel.target, wheelVelL(), wheelVelR(), dt, settings,
            balanceControl ? balanceAcceleration : NAN, recoveryControl);
    }
    currentLimited = currents.limited;
    leftWheel.current = currents.left; rightWheel.current = currents.right;
    leftPort.sendDriveRaw(m0601::ampsToRaw(currents.left * leftWheel.direction * P.curSign));
    rightPort.sendDriveRaw(m0601::ampsToRaw(currents.right * rightWheel.direction * P.curSign));
}

void HubDrive::reportFaults(HubWheel& w) {
    const uint8_t f = w.port->fb().fault;
    if (f == w.lastFault) return;
    w.lastFault = f;
    Serial.printf("[HUB] %s fault 0x%02X%s\n", w.port->name(), f,
        !f ? " cleared" : (f & 0x02) ? " BUS OVERCURRENT" : (f & 0x04) ? " PHASE OVERCURRENT" :
        (f & 0x08) ? " STALL" : (f & 0x10) ? " OVERHEAT" : (f & 0x01) ? " SENSOR" : "");
}

void HubDrive::printStatus() const {
    for (const auto* w : {&leftWheel, &rightWheel}) {
        const auto& f = w->port->fb();
        Serial.printf("[HUB] %s id%u mode%u rpm%d cmd%.2fA measured%.2fA fault%u age%lums CRC%lu timeout%lu\n",
            w->port->name(), w->port->id(), f.mode, f.rpm, w->current, f.currentA, f.fault,
            (unsigned long)(millis() - f.rxMs), (unsigned long)f.crcErrors, (unsigned long)f.timeouts);
    }
}
