#include "deployment_led.h"
#include <cassert>
#include <cstdint>
#include <cstdio>

int main() {
    HostLedLease lease;
    assert(!lease.active(0));
    assert(lease.update(1,0,1500,100));assert(lease.active(1599));assert(!lease.active(1600));
    assert(!lease.update(10,0,1000,100));assert(!lease.update(1,101,1000,100));assert(!lease.update(1,0,5000,100));
    assert(lease.update(3,20,1500,UINT32_MAX-500));assert(lease.active(100));assert(!lease.active(1200));
    assert(lease.update(0,0,1500,0));assert(!lease.active(1));
    for(unsigned phase=0;phase<=9;++phase)for(int i=0;i<16;++i) {
        float r,g,b;deploymentPixel(phase,50,12345.0f,i,16,r,g,b);
        assert(r>=0 && g>=0 && b>=0 && r<=.65f && g<=.65f && b<=.65f);
    }
    float r,g,b;deploymentPixel(8,0,0,0,16,r,g,b);assert(r>g && r>b);
    deploymentPixel(7,100,0,0,16,r,g,b);assert(g>r && g>b);
    puts("Deployment LEDs: lease expiry, rollover, validation and bounded status colors passed");
}
