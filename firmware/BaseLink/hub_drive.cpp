#include "hub_drive.h"

HubDrive hub;
HubDrive::HubDrive()
    : rightPort(Serial2, RIGHT_MOTOR_ID, RIGHT_RS485_RX, RIGHT_RS485_TX, RIGHT_RS485_DE, "RIGHT"),
      leftPort(Serial1, LEFT_MOTOR_ID, LEFT_RS485_RX, LEFT_RS485_TX, LEFT_RS485_DE, "LEFT"),
      rightWheel{&rightPort}, leftWheel{&leftPort} {}

void HubDrive::applyTuning() {
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

void HubDrive::service(float) { serviceWheel(leftWheel); serviceWheel(rightWheel); }
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
    lastDriveUs = 0;
}

void HubDrive::handshake(HubWheel& w) {
    if (!w.port->canSend()) return;
    if (w.modeOk) { w.port->sendDriveRaw(0); return; }
    if (w.queryNext) w.port->sendQuery();
    else w.port->sendMode(wantedMode());
    w.queryNext = !w.queryNext;
}

void HubDrive::transmit(float dt) {
    if (!modeOk() || faultBits()) {
        disable(); handshake(leftWheel); handshake(rightWheel); return;
    }
    if (!leftPort.canSend() || !rightPort.canSend()) return;
    if (!enabled && millis() - lastQueryMs >= 500) {
        // QUERY layout is never interleaved with armed DRIVE transactions.
        leftPort.sendQuery(); rightPort.sendQuery(); lastQueryMs = millis(); return;
    }
    const uint32_t now = micros();
    if (lastDriveUs) dt = (now - lastDriveUs) * 1e-6f;
    lastDriveUs = now;
    if (wantedMode() == m0601::MODE_VELOCITY) {
        const float scale = 60 / (2 * PI * P.wheelR);
        leftPort.sendDriveRaw(enabled ? lroundf(hub_control::clip(leftWheel.target * scale * leftWheel.direction, -330, 330)) : 0, (uint8_t)P.hubAcc);
        rightPort.sendDriveRaw(enabled ? lroundf(hub_control::clip(rightWheel.target * scale * rightWheel.direction, -330, 330)) : 0, (uint8_t)P.hubAcc);
        return;
    }
    hub_control::Currents currents;
    if (enabled) {
        const hub_control::Settings settings{P.hubKp, P.hubKi, P.hubIntLim, P.hubKff, P.hubMass, P.wheelR, M0601_KT,
            P.hubYawKp, P.hubYawKi, P.hubYawLim, P.hubYawSlew, P.motorMa * .001f};
        currents = controller.update(leftWheel.target, rightWheel.target, wheelVelL(), wheelVelR(), dt, settings);
    }
    leftWheel.current = currents.left; rightWheel.current = currents.right; currentLimited = currents.limited;
    leftPort.sendDriveRaw(m0601::ampsToRaw(currents.left * leftWheel.direction * P.curSign));
    rightPort.sendDriveRaw(m0601::ampsToRaw(currents.right * rightWheel.direction * P.curSign));
}

void HubDrive::printStatus() const {
    for (const auto* w : {&leftWheel, &rightWheel}) {
        const auto& f = w->port->fb();
        Serial.printf("[HUB] %s id%u mode%u rpm%d cmd%.2fA measured%.2fA fault%u age%lums CRC%lu timeout%lu\n",
            w->port->name(), w->port->id(), f.mode, f.rpm, w->current, f.currentA, f.fault,
            (unsigned long)(millis() - f.rxMs), (unsigned long)f.crcErrors, (unsigned long)f.timeouts);
    }
}
