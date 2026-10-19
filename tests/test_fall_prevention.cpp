// Compiles the production runController() with only its hardware dependencies
// stubbed. scripts/diagnostics/check_fall_prevention.py supplies that function.
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "../firmware/BaseLink/fall_prevention.h"
#include "../firmware/BaseLink/tof_guard.h"
#include "../firmware/BaseLink/auto_climb.h"
#include "../firmware/BaseLink/descent_guard.h"

struct FixtureParams {
    float climbVel=.20f, maxDriveV=.908784f, vMax=8500.0f/5092.96f, aMax=20000.0f/5092.96f;
    float maxTilt=55, driveDb=.03f, exClamp=.12f, climbZLim=.50f, zIntLim=.20f;
    float c1=-22.2369f,c2=-20.0745f,c3=-81.3063f,c4=-12.7029f,c5=-8.165f;
    float s1=-60.0226f,s2=-31.0364f,s3=-82.0431f,s4=-10.2571f,s5=-25.8199f;
    float brakeLook=.12f,vsatBleed=2,stepsPerM=5092.96f,maxSpdSt=8500;
    float climbStr=.5f,steerDb=.03f,maxSteerSt=2811.66f,steerSlew=15000;
    float lookMaxDeg=18,trackW=.155f,countsPerM=13037.97f,yawRelatch=3000;
    float yawTiltSus=12,yawKp=.8f,yawKd=.05f,yawMaxSt=1000,airFreeze=1;
} P;
constexpr float DEG_TO_RAD=3.14159265358979323846f/180.0f;
float k1=-7.3234f,k2=-7.4495f,k3=-42.9506f,k4=-6.6862f,k5=-2.1082f;
float pitchF=0,rateF=0,posM=0,velF=0,vCmd=0,xRef=0,zInt=0,uOut=0;
float cmdFwd=0,cmdSteer=0,encDiff=0,diffRateF=0,lookCmd=0,steerSteps=0,diffTarget=0;
float requestedDriveV=0,appliedBalanceScale=1,wheelTargetL=0,wheelTargetR=0,appliedSteerSteps=0;
bool climbMode=false,stiffHold=false,driveReferenceActive=false,fixtureAirborne=false;
float fixtureSoftening=.55f;
uint8_t satV=0,satA=0;
uint32_t fixtureMs=1000;
FallPrevention fallPrevention;
ToFGuard tofGuard;
AutoClimb autoClimb;
DescentGuard descentGuard;
float climbXRef=0,climbIntegral=0,climbDriveRef=0,climbRecovery=0;
bool climbReferenceInitialized=false;
void* gestSeq=nullptr;
struct FakeSerial { template<typename... Args> void printf(const char*,Args...) {} void println(const char*) {} } Serial;
struct FakeSteppers {
    int32_t left=0,right=0;
    void setSpeeds(int32_t l,int32_t r) { left=l;right=r; }
} steppers;
float clampf(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
bool terrain_airborne() { return fixtureAirborne; }
float terrain_soften() { return fixtureSoftening; }
uint32_t millis() { return fixtureMs; }
#include "controller_under_test.inc"

void resetFixture() {
    P=FixtureParams();fallPrevention.reset();tofGuard=ToFGuard();autoClimb=AutoClimb();descentGuard.reset();
    climbXRef=climbIntegral=climbDriveRef=climbRecovery=0;climbReferenceInitialized=false;gestSeq=nullptr;
    pitchF=rateF=posM=velF=vCmd=xRef=zInt=uOut=cmdFwd=cmdSteer=encDiff=diffRateF=lookCmd=steerSteps=diffTarget=0;
    requestedDriveV=wheelTargetL=wheelTargetR=appliedSteerSteps=0;appliedBalanceScale=1;
    climbMode=stiffHold=driveReferenceActive=fixtureAirborne=false;
    fixtureSoftening=.55f;fixtureMs=1000;
}
void finiteAndBounded() {
    assert(std::isfinite(uOut) && std::isfinite(vCmd) && std::isfinite(fallPrevention.effectiveRef));
    assert(std::fabs(uOut)<=P.aMax+1e-5f);
    assert(std::fabs(vCmd)<=P.vMax+1e-5f);
    assert(std::fabs(wheelTargetL)<=P.maxSpdSt+.002f);
    assert(std::fabs(wheelTargetR)<=P.maxSpdSt+.002f);
    assert(std::fabs((wheelTargetL+wheelTargetR)*.5f-vCmd*P.stepsPerM)<.002f);
}
void unitChecks() {
    // A full-stick step has bounded reference slew/jerk, in either direction.
    for(int sign : {-1,1}) {
        FallPrevention fp;
        float prior=0,priorAcceleration=0;
        for(int i=0;i<120;++i) {
            fp.update(sign*.9f,0,0,0,0,1.67f,3.927f,55,.005f,false);
            assert(std::fabs(fp.operatorRef-prior)<=1.2f*.005f+1e-6f);
            assert(std::fabs(fp.refAcceleration-priorAcceleration)<=6*.005f+1e-5f);
            prior=fp.operatorRef;priorAcceleration=fp.refAcceleration;
        }
        const float before=fp.operatorRef;
        fp.update(-sign*.9f,sign*.5f,0,0,0,1.67f,3.927f,55,.005f,false);
        assert(std::fabs(fp.operatorRef-before)<.0061f);
        assert(fp.reasons&FallPrevention::REVERSAL);
        assert(fp.fullBalanceFeedback);
        // Intentional virtual-acceleration reset prevents motion away from target.
        assert(sign*fp.refAcceleration<=0);
    }
    FallPrevention fp;
    fp.update(.9f,.4f,5,170,0,1.67f,3.927f,55,.005f,false);
    assert(fp.predictedTilt>24 && fp.risk==1 && fp.recovering);
    assert(fp.effectiveRef==.4f && fp.fullBalanceFeedback);
    assert(fp.allocateSteering(7000,1000,8500)==0);
    for(int i=0;i<49;++i)fp.update(0,0,0,0,0,1.67f,3.927f,55,.005f,false);
    assert(fp.recovering);
    for(int i=0;i<2;++i)fp.update(0,0,0,0,0,1.67f,3.927f,55,.005f,false);
    assert(!fp.recovering && fp.risk==0);
    fp.update(.9f,0,0,0,1.65f,1.67f,3.927f,55,.005f,false);
    assert(fp.reasons&FallPrevention::SPEED_MARGIN);
    assert(fp.risk>.9f);
    fp.reset();fp.update(.9f,.2f,0,0,0,1.67f,3.927f,55,.005f,true);
    assert(fp.operatorRef==0 && fp.risk==1 && !fp.fullBalanceFeedback);
    assert(fp.reasons&FallPrevention::AIRBORNE);
    fp.reset();
    for(float common : {-8500.f,-8300.f,-4000.f,0.f,4000.f,8300.f,8500.f})
        for(float yaw : {-3000.f,0.f,3000.f}) {
            float applied=fp.allocateSteering(common,yaw,8500);
            assert(std::fabs(common-applied)<=8500 && std::fabs(common+applied)<=8500);
        }
    fp.update(.7f,0,0,0,0,1.67f,3.927f,55,0,false);
    assert(fp.operatorRef==0 && std::isfinite(fp.effectiveRef));
}
void integrationChecks() {
    resetFixture();cmdFwd=1;runController(.005f);
    assert(appliedBalanceScale==1 && driveReferenceActive);
    finiteAndBounded();
    // Neutral input while still moving stays in braking/velocity mode.
    cmdFwd=0;velF=.4f;runController(.005f);assert(driveReferenceActive);
    velF=0;fallPrevention.reset();runController(.005f);assert(!driveReferenceActive);
    // Stiff hold keeps its stronger gain set even when recovery is latched.
    resetFixture();stiffHold=true;pitchF=8;rateF=150;P.aMax=100;runController(.005f);
    assert(!driveReferenceActive && appliedBalanceScale==1);
    float expected=clampf(-(P.s3*pitchF*DEG_TO_RAD+P.s4*rateF*DEG_TO_RAD),-P.aMax,P.aMax);
    assert(std::fabs(uOut-expected)<1e-5f);finiteAndBounded();
    // Calm normal hold retains terrain adaptation.
    resetFixture();runController(.005f);assert(appliedBalanceScale==.55f);
    // Risk allocation must not overwrite the steering slew state every cycle.
    resetFixture();cmdSteer=1;vCmd=P.vMax*.9f;velF=vCmd;runController(.005f);
    assert(steerSteps==75 && appliedSteerSteps<steerSteps);finiteAndBounded();
    runController(.005f);assert(steerSteps==150);finiteAndBounded();
    // Stale guard clips a previously accumulated operator reference immediately,
    // while the LQR remains free to move wheels to recover pitch.
    resetFixture();fallPrevention.operatorRef=.7f;cmdFwd=1;pitchF=8;rateF=100;
    tofGuard.update(1,.2f,100);runController(.005f);
    assert(fallPrevention.operatorRef==0 && std::fabs(vCmd)>0);finiteAndBounded();
    resetFixture();autoClimb.select(AutoClimb::FORCE_CLIMB);autoClimb.blend=1;
    climbReferenceInitialized=true;cmdFwd=1;pitchF=5;rateF=170;climbXRef=.1f;climbIntegral=.1f;
    runController(.005f);assert(climbXRef<.1f && climbIntegral<.1f && appliedBalanceScale==1);finiteAndBounded();
    resetFixture();autoClimb.select(AutoClimb::FORCE_CLIMB);autoClimb.blend=1;
    P.climbVel=0;tofGuard.update(1,0,fixtureMs);
    runController(.005f);finiteAndBounded();
    resetFixture();fixtureAirborne=true;cmdFwd=1;cmdSteer=1;runController(.005f);
    assert(fallPrevention.operatorRef==0 && appliedSteerSteps==0);finiteAndBounded();
    // Reproduce the pg-to-home regression: mild latch must not wash out the
    // slope integral. The stronger C state remains independent of normal hold.
    resetFixture();autoClimb.select(AutoClimb::FORCE_CLIMB);autoClimb.blend=1;
    P.exClamp=.15f;P.climbZLim=2;climbReferenceInitialized=true;
    climbXRef=.15f;climbIntegral=-.6f;cmdFwd=1;pitchF=5;rateF=60;
    fallPrevention.recovering=true;runController(.005f);
    assert(fallPrevention.risk==.25f && climbRecovery==0 && climbIntegral<=-.6f);
    finiteAndBounded();
    // Do not wind the integral further into the negative acceleration clamp.
    resetFixture();autoClimb.select(AutoClimb::FORCE_CLIMB);autoClimb.blend=1;
    P.exClamp=.15f;P.climbZLim=2;climbReferenceInitialized=true;
    climbXRef=.15f;climbIntegral=-.9f;cmdFwd=1;runController(.005f);
    assert(climbIntegral==-.9f && uOut==-P.aMax);finiteAndBounded();
    // A manual exit fades gains without immediately discarding climb state.
    autoClimb.select(AutoClimb::FORCE_NORMAL);runController(.005f);
    assert(autoClimb.blend>.9f && autoClimb.blend<1 && climbReferenceInitialized);
    finiteAndBounded();
    // Feed a sustained low-progress recorded-state scenario through production
    // arbitration/transition code. This checks handover, not plant stability.
    resetFixture();cmdFwd=1;pitchF=5.5f;velF=.1f;fallPrevention.operatorRef=P.maxDriveV;
    bool entered=false;float previousBlend=0,previousU=0;
    for(int i=0;i<900;++i) {
        vCmd=.1f;wheelTargetL=wheelTargetR=.1f*P.stepsPerM;
        runController(.005f);finiteAndBounded();
        assert(std::fabs(autoClimb.blend-previousBlend)<=.005f/.75f+1e-6f);
        if(autoClimb.blend>0 && previousBlend==0) {
            assert(std::fabs(uOut-previousU)<.12f);entered=true;
        }
        assert(std::fabs(climbIntegral)<=P.climbZLim+1e-6f);
        previousBlend=autoClimb.blend;previousU=uOut;
    }
    assert(entered && autoClimb.active && autoClimb.blend==1);
}
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;std::istringstream stream(line);std::string value;
    while(std::getline(stream,value,','))fields.push_back(value);
    return fields;
}
void replay(const char* path) {
    std::ifstream input(path);assert(input.good());std::string line;std::getline(input,line);
    auto names=split(line);
    auto column=[&](const char* name) { for(size_t i=0;i<names.size();++i)if(names[i]==name)return i;assert(false);return size_t(0); };
    const auto timeCol=column("firmware_debug_ms"),balCol=column("balancing"),pitchCol=column("forward_pitch_deg"),
        rateCol=column("pitch_rate_dps"),velocityCol=column("filtered_speed_mps"),cmdCol=column("base_cmd_steps_s"),
        fwdCol=column("forward_input"),yawCol=column("steer_input"),sfCol=column("softening"),terrainCol=column("terrain_state");
    bool previouslyBalancing=false;float priorMs=0;unsigned samples=0,fullFeedback=0,recoveries=0,reversals=0;
    float peakReferenceStep=0;
    resetFixture();
    while(std::getline(input,line)) {
        auto cells=split(line);if(cells.size()<names.size())continue;
        bool balancing=cells[balCol]=="True" || cells[balCol]=="1";
        if(!balancing) { previouslyBalancing=false;continue; }
        if(!previouslyBalancing)resetFixture();
        float ms=std::stof(cells[timeCol]);float dt=previouslyBalancing?clampf((ms-priorMs)/1000,0,.05f):.005f;
        priorMs=ms;previouslyBalancing=true;
        pitchF=std::stof(cells[pitchCol]);rateF=std::stof(cells[rateCol]);velF=std::stof(cells[velocityCol]);
        vCmd=std::stof(cells[cmdCol])/P.stepsPerM;cmdFwd=std::stof(cells[fwdCol]);cmdSteer=std::stof(cells[yawCol]);
        fixtureSoftening=std::stof(cells[sfCol]);fixtureAirborne=std::stof(cells[terrainCol])==2;
        float priorRef=fallPrevention.operatorRef;runController(dt);finiteAndBounded();
        if(!fixtureAirborne)assert(std::fabs(fallPrevention.operatorRef-priorRef)<=1.2f*dt+1e-5f);
        peakReferenceStep=std::fmax(peakReferenceStep,std::fabs(fallPrevention.operatorRef-priorRef));
        if(fallPrevention.fullBalanceFeedback)assert(appliedBalanceScale==1);
        ++samples;fullFeedback+=fallPrevention.fullBalanceFeedback;recoveries+=fallPrevention.recovering;
        reversals+=(fallPrevention.reasons&FallPrevention::REVERSAL)!=0;
    }
    assert(samples>1000 && reversals>0 && recoveries>0);
    std::cout<<"Recorded-state replay: "<<samples<<" samples, "<<fullFeedback<<" full-feedback, "
             <<recoveries<<" recovery, "<<reversals<<" reversal; maximum reference step "<<peakReferenceStep<<" m/s\n";
    std::cout<<"Replay checks constraints at recorded states; it is not a closed-loop stability prediction.\n";
}
int main(int argc,char** argv) {
    unitChecks();integrationChecks();std::cout<<"Fall-prevention unit and production-controller integration checks passed\n";
    if(argc==2)replay(argv[1]);
}
