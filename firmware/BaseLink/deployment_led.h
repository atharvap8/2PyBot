#pragma once
#include <cmath>
#include <stdint.h>

// Shared numeric vocabulary with console/deployment_status.py.
struct HostLedLease {
    uint8_t phase = 0, progress = 0;
    uint32_t expires = 0;
    bool update(unsigned next, unsigned percent, unsigned ttl, uint32_t now) {
        if (next > 9 || percent > 100 || ttl < 250 || ttl > 3000) return false;
        phase = (uint8_t)next; progress = (uint8_t)percent;
        expires = now + ttl;
        return true;
    }
    bool active(uint32_t now) const { return phase && (int32_t)(expires - now) > 0; }
};

inline void deploymentPixel(unsigned phase, unsigned progress, float seconds, int index, int count,
                            float& red, float& green, float& blue) {
    (void)progress; // Progress stays in the API; the ring is only a blended spin.
    if (!phase || phase > 9 || count <= 0) { red=green=blue=0; return; }
    float first[3]={.02f,.75f,1},second[3]={.6f,.12f,1};
    if(phase==7) { first[0]=.06f;first[1]=1;first[2]=.25f;second[0]=.03f;second[1]=.45f;second[2]=.08f; }
    else if(phase==8) { first[0]=1;first[1]=.05f;first[2]=.02f;second[0]=.45f;second[1]=.01f;second[2]=.01f; }
    else if(phase==9) { first[0]=1;first[1]=.5f;first[2]=.05f;second[0]=1;second[1]=.15f;second[2]=.01f; }
    const float angle=6.2831853f*(index/(float)count-seconds/4);
    const float blend=.5f+.5f*std::sin(angle);
    const float level=.25f+.65f*(.5f+.5f*std::cos(angle));
    red=(first[0]*(1-blend)+second[0]*blend)*level*.65f;
    green=(first[1]*(1-blend)+second[1]*blend)*level*.65f;
    blue=(first[2]*(1-blend)+second[2]*blend)*level*.65f;
}
