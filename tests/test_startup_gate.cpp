#include <cassert>
#include "../firmware/BaseLink/startup_gate.h"

int main() {
    StartupGate gate;
    assert(!gate.mayInitialize(true));
    assert(!gate.mayArm(true));
    // Arbitrarily long host boot must not release initialization by timeout.
    for (int second = 0; second < 180; ++second) assert(!gate.mayInitialize(true));
    gate.acknowledge(true);
    assert(gate.mayInitialize(true));
    assert(!gate.mayArm(true)); // ACK is not calibration completion or arming.
    gate.acknowledge(true); // retries are idempotent
    assert(!gate.initialized);
    gate.complete();
    assert(gate.mayArm(true));
    gate.acknowledge(false);
    assert(!gate.mayArm(true));
    StartupGate standalone;
    assert(standalone.mayInitialize(false));
    assert(!standalone.mayArm(false));
    standalone.complete();
    assert(standalone.mayArm(false));
    standalone.maintenance=true;
    assert(!standalone.mayArm(false)); // flash lock also protects standalone mode
    assert(!standalone.mayArm(true));
    standalone.acknowledge(true);assert(!standalone.mayArm(true));
    standalone.maintenance=false;assert(standalone.mayArm(false));
}
