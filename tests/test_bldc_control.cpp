#include "hub_control.h"
#include <cassert>
#include <cstdio>
#include <deque>
using namespace hub_control;
static Settings settings{6,8,1,1,2,.052f,.75f,1.5f,2,.35f,6,2.7f};

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
    assert(std::fabs(loop.commonIntegral)<=1.00001f && std::fabs(loop.yawIntegral)<=.35001f);
    Steering steer; steer.reset(0);
    float prev=0;
    for(int tick=0;tick<200;++tick) {
        float result=steer.update(tick<80?1000:0,tick<80,tick*10,2000,0,1.5f,.03f,800,2000,false,4000,900,200,.005f);
        assert(std::fabs(result-prev)<=20.001f); prev=result;
    }
    assert(std::fabs(prev)<1); // release decelerates before heading latch
    for(float gain:{20.0f,40.0f,70.0f}) for(int delay:{1,2}) stepResponse(gain,delay,true);
    puts("BLDC control: balance priority, turn symmetry/slew and delayed turn step responses passed");
}
