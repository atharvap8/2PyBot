#pragma once
#include <cmath>

// Soft descent-reference regulator. Never clamps actual wheel speed, changes
// motor enable, or reduces the LQR's physical recovery acceleration ceiling.
struct DescentGuard {
    bool active=false,seeded=false;
    float scale=1.0f,limit=0,excess=0,goal=0,velocityMean=0;
    void reset() { *this=DescentGuard(); }
    static float clip(float x,float lo,float hi) { return x<lo?lo:x>hi?hi:x; }
    void update(float nominalReference,float measuredVelocity,bool eligible,bool descentCue,bool airborne,float dt) {
        dt=clip(dt,0,.05f);
        if(!std::isfinite(nominalReference) || !std::isfinite(measuredVelocity))return;
        if(!seeded) { velocityMean=measuredVelocity;seeded=true; }
        else velocityMean+=dt/(.15f+dt)*(measuredVelocity-velocityMean);
        limit=std::fabs(nominalReference);
        const float direction=limit>.02f?(nominalReference>0?1.0f:-1.0f):(velocityMean>=0?1.0f:-1.0f);
        excess=std::fmax(0.0f,direction*velocityMean-limit);
        active=eligible && descentCue && !airborne;
        const float reduction=active?clip((excess-.06f)/std::fmax(.15f,.80f*limit),0,1):0;
        const float target=1.0f-reduction;
        scale+=clip(target-scale,-2.0f*dt,.50f*dt);
        scale=clip(scale,0,1);
        if(nominalReference*velocityMean<0) { scale=1;goal=nominalReference; }
        else goal=nominalReference*scale;
    }
    float limitReference(float reference) const {
        if(reference*velocityMean<0)return reference; // never attenuate deliberate reverse braking
        return active?clip(reference,-limit,limit)*scale:reference*scale;
    }
};
