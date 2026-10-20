#include "hub_control.h"
#include "fall_prevention.h"
#include <cassert>
#include <cstdio>
#include <deque>
using namespace hub_control;
static Settings settings{4,4,.5f,1,2,.052f,.75f,1.5f,2,.35f,6,2.7f};

static void stepResponse(float gain,int delay,bool yaw) {
    CurrentLoop loop;
    float velocity=0,integral=0,peak=0;
    std::deque<float> commands(delay,0);
    for(int tick=0;tick<2000;++tick) {
        const float target=tick>=20?.15f:0;
        const float measured=std::round(velocity/.005445f)*.005445f; // 1 rpm at 104 mm OD
        auto i=loop.update(yaw?-target:target,target,yaw?-measured:measured,measured,.005f,settings);
        assert(std::fabs(i.left)<=2.7001f && std::fabs(i.right)<=2.7001f);
        commands.push_back(yaw?.5f*(i.right-i.left):.5f*(i.right+i.left));
        const float applied=commands.front(); commands.pop_front();
        velocity+=.005f*(gain*applied-.1f*velocity);
        peak=std::fmax(peak,std::fabs(velocity));
        if(tick>=1800) integral+=std::fabs(velocity-target)/200;
    }
    assert(peak<.45f && integral<.01f);
}

static void delayedCommonResponse(float gain,int delay) {
    CurrentLoop loop;
    float velocity=0,feedback=0,target=0,tailError=0,peak=0;
    std::deque<float> commands(delay,0);
    for(int tick=0;tick<2000;++tick) {
        float next=tick<20?0:std::fmin(.3f,target+.005f);
        float acceleration=(next-target)/.005f;target=next;
        feedback+=(1-std::exp(-2*3.14159265f*30*.005f))*(velocity-feedback);
        auto i=loop.update(target,target,feedback,feedback,.005f,settings,acceleration,false);
        commands.push_back(.5f*(i.left+i.right));float applied=commands.front();commands.pop_front();
        velocity+=.005f*(gain*applied-.1f*velocity);peak=std::fmax(peak,std::fabs(velocity));
        if(tick>=1800)tailError+=std::fabs(velocity-target)/200;
        assert(std::fabs(i.left)<=2.7001f && std::fabs(i.right)<=2.7001f);
    }
    assert(peak<.65f && tailError<.01f);
}

int main() {
    CurrentLoop loop;
    for(int tick=0;tick<500;++tick) {
        auto i=loop.update(-.3f,.3f,0,0,.005f,settings);
        assert(std::fabs(i.left+i.right)<.00001f); // pure turn never changes common torque
    }
    for(int tick=0;tick<200;++tick) {
        auto i=loop.update(.9f,1.5f,0,0,.005f,settings);
        assert(std::fabs(.5f*(i.left+i.right)-2.7f)<.0001f); // balance wins over yaw
        assert(std::fabs(i.left)<=2.7001f && std::fabs(i.right)<=2.7001f);
    }
    assert(std::fabs(loop.commonIntegral)<=.50001f && std::fabs(loop.yawIntegral)<=.35001f);
    // Captured failure: target stuck at 0.725 m/s, wheels already at 0.806,
    // positive catch acceleration. The speed PI must not command braking.
    loop.reset(.725f);
    auto catchForward=loop.update(.725f,.725f,.806f,.806f,.005f,settings,6,true);
    assert(catchForward.left>0 && catchForward.right>0 && loop.recoveryPriority);
    assert(std::fabs(loop.feedForwardCurrent-6*2*.052f/(2*.75f))<.00001f);
    loop.reset(-.725f);
    auto catchReverse=loop.update(-.725f,-.725f,-.806f,-.806f,.005f,settings,-6,true);
    assert(catchReverse.left<0 && catchReverse.right<0 && loop.recoveryPriority);
    // A constant (explicitly limited) speed target cannot erase acceleration FF.
    loop.reset(1);
    auto atLimit=loop.update(1,1,1,1,.005f,settings,6,true);
    assert(atLimit.left>.4f && atLimit.right>.4f);
    // No absolute 12 V/model speed cap in the virtual actuator state.
    auto aboveModel=integrateVelocity(1.1f,6,1.1f,.005f,settings.kp,settings.currentLimit,0);
    assert(aboveModel.value>1.1f && !aboveModel.trackingLimited);
    float stalled=0;
    for(int tick=0;tick<1000;++tick)stalled=integrateVelocity(stalled,6,0,.005f,settings.kp,settings.currentLimit,0).value;
    assert(stalled<=settings.currentLimit/settings.kp+.00001f);
    auto moving=integrateVelocity(1.5f,6,1.5f,.005f,settings.kp,settings.currentLimit,0);
    assert(moving.value>1.5f); // tracking anti-windup is not a vehicle speed limit
    auto explicitLimit=integrateVelocity(1,6,1,.005f,settings.kp,settings.currentLimit,1);
    assert(explicitLimit.value==1 && explicitLimit.trackingLimited);
    FallPrevention reference;
    for(int tick=0;tick<400;++tick)reference.update(1,1,0,0,1.2f,0,6,55,.005f,false);
    assert(std::fabs(reference.operatorRef-1)<.00001f && std::fabs(reference.effectiveRef-1)<.00001f);
    assert(!(reference.reasons & FallPrevention::SPEED_MARGIN));
    assert(reference.allocateSteering(20000,1000,0)==1000);
    Steering steer; steer.reset(0);
    float prev=0;
    for(int tick=0;tick<200;++tick) {
        float result=steer.update(tick<80?1000:0,tick<80,tick*10,2000,0,1.5f,.03f,800,2000,false,4000,900,200,.005f);
        assert(std::fabs(result-prev)<=20.001f); prev=result;
    }
    assert(std::fabs(prev)<1); // release decelerates before heading latch
    for(float gain:{20.0f,40.0f,70.0f}) for(int delay:{1,2}) stepResponse(gain,delay,true);
    for(float gain:{8.0f,14.0f,25.0f}) for(int delay:{1,2}) delayedCommonResponse(gain,delay);
    puts("BLDC control: balance priority, turn symmetry/slew and delayed turn step responses passed");
}
