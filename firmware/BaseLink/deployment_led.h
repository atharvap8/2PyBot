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
    const float colors[10][3] = {{0,0,0},{.02f,.75f,1},{.15f,.35f,1},{.6f,.12f,1},
        {.3f,.25f,1},{1,.45f,.02f},{.03f,1,.65f},{.06f,1,.25f},{1,.05f,.02f},{1,.5f,.05f}};
    if (phase > 9 || count <= 0) { red=green=blue=0; return; }
    float level;
    if (phase == 5) {
        const float position = progress * count / 100.0f;
        level = index < position ? .65f : .025f;
        const int tip = (int)position < count ? (int)position : count-1;
        if (index == tip) level=.65f+.15f*(1+std::sin(seconds*6));
    } else if (phase == 7) level=.6f+.25f*std::cos(index*6.2831853f/count-seconds*4);
    else if (phase == 8) level=index%4!=3 ? .2f+.3f*(1+std::sin(seconds*3)) : .02f;
    else if (phase == 9) level=index%4==0 ? .08f+.325f*(1+std::sin(seconds*2)) : .02f;
    else {
        const float position=seconds*(phase==1 ? 7 : 4);
        float a=std::fmod(index-position,(float)count); if(a<0)a+=count;
        float b=std::fmod(index-position-count*.5f,(float)count); if(b<0)b+=count;
        level=.035f+.8f*std::fmax(std::exp(-a*.8f),std::exp(-b*.8f));
    }
    red=colors[phase][0]*level*.65f;
    green=colors[phase][1]*level*.65f;
    blue=colors[phase][2]*level*.65f;
}
