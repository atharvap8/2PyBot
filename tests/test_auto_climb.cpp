#include <cassert>
#include <cmath>
#include <iostream>
#define DISABLED 0x00 // Arduino GPIO headers reserve this macro
#define constrain(x,a,b) ((x)<(a)?(a):(x)>(b)?(b):(x))
#include "../firmware/BaseLink/auto_climb.h"
#include "../firmware/BaseLink/fall_prevention.h"
#include "../firmware/BaseLink/descent_guard.h"

void step(AutoClimb& ac,float goal=.9f,float velocity=.1f,float pitch=5.5f,float rate=0,
          float steering=0,float command=.1f,float hazard=0,bool airborne=false,bool capped=false) {
    ac.update(goal,velocity,pitch,rate,steering,command,command,velocity,velocity,.908784f,
              hazard,airborne,false,capped,.005f);
    assert(std::isfinite(ac.blend) && ac.blend>=0 && ac.blend<=1);
}
void run(AutoClimb& ac,int count,float goal=.9f,float velocity=.1f,float pitch=5.5f,
         float rate=0,float steering=0,float command=.1f,float hazard=0,bool airborne=false) {
    for(int i=0;i<count;++i)step(ac,goal,velocity,pitch,rate,steering,command,hazard,airborne);
}
int main() {
    AutoClimb ac;
    run(ac,400,.9f,.78f,2.3f,0,0,.88f);assert(!ac.active && ac.blend==0);
    run(ac,350);assert(ac.active);run(ac,160);assert(ac.blend==1);
    // A brief neutral pause while on a slope must retain compensation.
    run(ac,400,0,0,4);assert(ac.active && ac.blend==1);
    // Flat, healthy tracking needs sustained exit confirmation (not one sample).
    run(ac,500,.25f,.25f,.55f,0,0,.25f);assert(ac.active);
    run(ac,1100,.25f,.25f,.55f,0,0,.25f);assert(!ac.active && ac.blend==0);
    ac=AutoClimb();run(ac,650);assert(ac.active);
    run(ac,1600,.125f,.125f,.275f,0,0,.125f);
    assert(!ac.active && ac.blend==0); // LOW-mode climb must also return to normal
    // Both direction signs and overspeed load-compensation candidates.
    ac=AutoClimb();run(ac,650,-.9f,-.1f,-5.5f,0,0,-.1f);assert(ac.active);
    ac=AutoClimb();run(ac,650,.25f,.60f,-3.0f,0,0,.60f);assert(ac.active);
    assert(ac.kind==AutoClimb::DOWN_LOAD && ac.state==AutoClimb::DOWNCLIMB);
    ac=AutoClimb();run(ac,650,-.25f,-.60f,3.0f,0,0,-.60f);
    assert(ac.active && ac.kind==AutoClimb::DOWN_LOAD);
    ac=AutoClimb();run(ac,650);run(ac,2200,0,0,0,0,0,0);
    assert(!ac.active && ac.blend==0); // flat neutral release must not latch indefinitely
    ac=AutoClimb();run(ac,650);run(ac,2200,.125f,.125f,.275f,0,.40f,.125f);
    assert(!ac.active && ac.blend==0); // moderate steering must not block release forever
    ac=AutoClimb();run(ac,500,.9f,.04f,3.3f,0,0,.04f);assert(!ac.active);
    run(ac,300,.9f,.04f,3.3f,0,0,.04f);assert(ac.active); // strong loss / smaller bias needs longer confirmation
    ac=AutoClimb();run(ac,650);assert(ac.active);
    run(ac,2100,.25f,0,5.5f,0,0,0);
    assert(ac.active && ac.state==AutoClimb::LIMITED && (ac.flags&AutoClimb::NO_PROGRESS));
    // Sparse acceleration caps slow flat confirmation without chattering out.
    ac=AutoClimb();run(ac,650);assert(ac.active);
    for(int i=0;i<500;++i)step(ac,.25f,.25f,.55f,0,0,.25f,0,false,i%6==0);
    assert(ac.active);
    for(int i=0;i<1400;++i)step(ac,.25f,.25f,.55f,0,0,.25f,0,false,i%6==0);
    assert(!ac.active && ac.blend==0);
    // The old exit failed whenever balance acceleration capped repeatedly.
    ac=AutoClimb();run(ac,650);
    for(int i=0;i<2200;++i)step(ac,.125f,.125f,.275f,0,0,.125f,0,false,i%2==0);
    assert(!ac.active && ac.blend==0);
    // A failed hand-back must preserve assistance rather than force an exit.
    ac=AutoClimb();run(ac,650);bool probing=false;
    for(int i=0;i<2000 && !ac.exitPending;++i)step(ac,.125f,.125f,.275f,0,0,.125f);
    assert(ac.exitPending);probing=true;
    run(ac,400,.45f,.02f,5.5f,0,0,.02f);
    assert(probing && ac.active && !ac.exitPending && ac.blend==1);
    // Large actual command with little encoder motion is an actuator/load flag,
    // not permission to increase climb compensation automatically.
    ac=AutoClimb();run(ac,800,.9f,0,5.5f,0,0,.8f);
    assert(ac.actuatorLimited && ac.state==AutoClimb::LIMITED && !ac.active);
    ac=AutoClimb();run(ac,800,.9f,.1f,5.5f,0,.8f);assert(!ac.active);
    ac=AutoClimb();run(ac,800,.9f,.1f,5.5f,0,0,.1f,.8f);assert(!ac.active);
    ac=AutoClimb();run(ac,800,.9f,.1f,5.5f,0,0,.1f,0,true);assert(!ac.active);
    ac=AutoClimb();run(ac,800,.9f,.1f,5.5f,130);assert(!ac.active);
    // Rapid forward/reverse pulses never meet steady-request confirmation.
    ac=AutoClimb();for(int i=0;i<30;++i)run(ac,80,i%2?.9f:-.9f,0,i%2?5.5f:-5.5f);
    assert(!ac.active);
    // Manual selection wins until auto is explicitly restored.
    ac=AutoClimb();ac.select(AutoClimb::FORCE_NORMAL);run(ac,1000);assert(!ac.active && ac.blend==0);
    ac.resetDynamics();run(ac,800);assert(ac.policy==AutoClimb::FORCE_NORMAL && !ac.active);
    ac.select(AutoClimb::FORCE_CLIMB);run(ac,160);assert(ac.blend==1 && ac.state==AutoClimb::MANUAL);
    ac.configure(false);run(ac,160);assert(ac.blend==0 && ac.state==AutoClimb::AUTO_DISABLED);
    ac.resetDynamics();assert(!ac.enabled);ac.configure(true);run(ac,650);assert(ac.active);
    // Invalid observations cannot create automatic entry.
    ac=AutoClimb();step(ac,NAN);assert(!ac.active && ac.blend==0);
    FallPrevention fp;fp.recovering=true;
    fp.update(.25f,0,3,60,0,1.669f,3.927f,55,.005f,false);
    assert(fp.risk==.25f && fp.hazardRisk==0 && fp.climbRecoveryScale()==0);
    fp.update(.25f,0,20,100,0,1.669f,3.927f,55,.005f,false);
    assert(fp.climbRecoveryScale()==1);
    for(int sign : {-1,1}) {
        DescentGuard guard;
        for(int i=0;i<150;++i)guard.update(sign*.25f,sign*.60f,true,true,false,.005f);
        assert(guard.active && guard.scale<.01f && std::fabs(guard.goal)<.003f);
        assert(std::fabs(guard.limitReference(sign*.9f))<=.25f);
        guard.update(-sign*.25f,sign*.6f,true,true,false,.005f);
        assert(std::fabs(guard.goal+sign*.25f)<1e-6f); // preserve intentional reverse braking
        assert(std::fabs(guard.limitReference(-sign*.9f)+sign*.9f)<1e-6f);
        guard.update(sign*.25f,sign*.1f,false,false,false,.005f);
        assert(!guard.active);
    }
    std::cout<<"Automatic climb detection, manual overrides, hysteresis and hazard-specific recovery checks passed\n";
}
