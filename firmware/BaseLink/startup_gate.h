#pragma once

// Host acknowledgement releases initialization; it never requests motor arming.
class StartupGate {
public:
    bool hostReady = false;
    bool initialized = false;
    bool maintenance = false;

    void acknowledge(bool ready) { hostReady = ready; }
    bool mayInitialize(bool requireHost) const { return !requireHost || hostReady; }
    void complete() { initialized = true; }
    bool mayArm(bool requireHost) const { return initialized && !maintenance && (!requireHost || hostReady); }
};
