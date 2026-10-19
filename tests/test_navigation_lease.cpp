#include <cassert>
#include <cmath>
#include <iostream>
#include "../firmware/BaseLink/navigation_lease.h"

int main() {
    NavigationLease lease;
    assert(!lease.grant(1,100,false,true,true)); // cannot remotely arm
    assert(!lease.grant(1,100,true,false,true));
    assert(!lease.grant(1,100,true,true,false));
    assert(lease.grant(1,100,true,true,true));
    assert(!lease.command(2,1,.1f,0,110));
    assert(!lease.command(1,1,-.1f,0,110)); // no autonomous reverse
    assert(!lease.command(1,1,.7f,0,110));
    assert(!lease.command(1,1,.1f,NAN,110));
    assert(lease.command(1,1,.1f,.4f,110));
    assert(lease.command(1,2,.6f,0,110));
    assert(!lease.command(1,1,.1f,0,200));
    assert(!lease.command(1,0,.1f,0,200));
    assert(lease.lastRx==110);
    assert(!lease.update(611,false,true) && lease.state==NavigationLease::NAV_TIMEOUT);
    assert(lease.velocity==0 && lease.yawRate==0);
    assert(!lease.command(1,2,.1f,0,620));
    assert(!lease.grant(1,620,true,true,true)); // cancelled token cannot resume
    assert(lease.grant(2,630,true,true,true));
    assert(!lease.update(640,true,true) && lease.state==NavigationLease::NAV_TAKEOVER);
    assert(!lease.command(2,3,.1f,0,650));
    assert(lease.grant(3,UINT32_MAX-100,true,true,true));
    assert(lease.command(3,UINT32_MAX-1,.1f,0,UINT32_MAX-50));
    assert(lease.command(3,0,.1f,0,10)); // sequence/millis rollover
    assert(lease.update(400,false,true));
    assert(!lease.update(511,false,true));
    assert(lease.grant(4,600,true,true,true));
    assert(!lease.update(601,false,false));
    std::cout<<"Navigation lease: no remote arm, bounds, stale packets, takeover and rollover passed\n";
}
