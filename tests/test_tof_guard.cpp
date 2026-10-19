#include <cassert>
#include <cmath>
#include "../firmware/BaseLink/tof_guard.h"

int main() {
    ToFGuard guard;
    assert(guard.apply(1, .7f, 1000) == 1); // disabled is transparent
    assert(guard.update(1, .35f, 1000));
    assert(std::fabs(guard.apply(1, .7f, 1100) - .5f) < 1e-6);
    assert(guard.apply(-1, .7f, 1100) == -1); // backing away remains possible
    assert(guard.apply(1, .7f, 1600) == 0); // enabled link loss blocks requests
    assert(!guard.update(1, NAN, 2000));
    assert(!guard.update(2, .7f, 2000));
    assert(!guard.update(1, -1, 2000));
    assert(guard.update(1, .35f, UINT32_MAX - 100));
    assert(std::fabs(guard.apply(1, .7f, 50) - .5f) < 1e-6); // millis rollover
    assert(guard.update(0, 0, 3000));
    assert(guard.apply(1, .7f, 5000) == 1);
}
