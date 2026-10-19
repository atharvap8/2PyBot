#pragma once
#include <cmath>
#include <stdint.h>

// Separate motion lease: it cannot enable motors and stale/replayed packets
// cannot re-acquire authority after gamepad takeover, timeout or a stop.
struct NavigationLease {
    enum State : uint8_t { NAV_OFF=0,NAV_GRANTED=1,NAV_DRIVING=2,NAV_TAKEOVER=3,NAV_TIMEOUT=4,NAV_STOPPED=5,NAV_DISABLED=6 };
    bool granted=false,haveSequence=false;
    uint32_t token=0,sequence=0,lastRx=0;
    float velocity=0,yawRate=0;
    State state=NAV_OFF;
    void reset() { *this=NavigationLease(); }
    bool grant(uint32_t next,uint32_t now,bool balancing,bool available,bool neutral) {
        if(!next || !balancing || !available || !neutral)return false;
        if(next==token)return granted; // a cancelled token is never reusable
        token=next;sequence=0;haveSequence=false;lastRx=now;
        velocity=yawRate=0;granted=true;state=NAV_GRANTED;return true;
    }
    bool command(uint32_t owner,uint32_t seq,float v,float yaw,uint32_t now) {
        if(!granted || owner!=token || !std::isfinite(v) || !std::isfinite(yaw))return false;
        if(haveSequence && (int32_t)(seq-sequence)<=0)return false;
        sequence=seq;haveSequence=true;lastRx=now;velocity=v;yawRate=yaw;state=NAV_DRIVING;return true;
    }
    void cancel(State reason) { granted=false;velocity=yawRate=0;state=reason; }
    bool update(uint32_t now,bool takeover,bool eligible) {
        if(granted && takeover)cancel(NAV_TAKEOVER);
        else if(granted && !eligible)cancel(NAV_DISABLED);
        else if(granted && (uint32_t)(now-lastRx)>500)cancel(NAV_TIMEOUT);
        return granted;
    }
};
