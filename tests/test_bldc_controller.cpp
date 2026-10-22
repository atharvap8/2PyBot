// Compile the actual runController(), not a reimplementation of its policy.
#include "params.h"
#include "hub_drive.h"
#include "fall_prevention.h"
#include "tof_guard.h"
#include "auto_climb.h"
#include "descent_guard.h"
#include "navigation_lease.h"
#include <cassert>

constexpr float DEG_TO_RAD=PI/180;
#define k1 P.k1
#define k2 P.k2
#define k3 P.k3
#define k4 P.k4
#define k5 P.k5
float pitchF=0,rateF=0,posM=0,velF=0,vCmd=0,xRef=0,zInt=0,uOut=0;
float cmdFwd=0,cmdSteer=0,encDiff=0,diffRateF=0,lookCmd=0,steerSteps=0,diffTarget=0;
float requestedDriveV=0,appliedBalanceScale=1,wheelTargetL=0,wheelTargetR=0,appliedSteerSteps=0;
float climbXRef=0,climbIntegral=0,climbDriveRef=0,climbRecovery=0;
bool climbMode=false,stiffHold=false,driveReferenceActive=false,climbReferenceInitialized=false;
uint8_t satV=0,satA=0;
void* gestSeq=nullptr;
FallPrevention fallPrevention;
ToFGuard tofGuard;
AutoClimb autoClimb;
DescentGuard descentGuard;
NavigationLease navigation;
hub_control::Steering yawControl;
bool terrain_airborne() { return false; }
float testTerrainScale=1;
float terrain_soften() { return testTerrainScale; }
float clampf(float value,float lo,float hi) { return hub_control::clip(value,lo,hi); }
bool params_motionActive() { return false; }
void params_applyHardware(bool) { hub.applyTuning(); }
#include "bldc_controller_under_test.inc"

void reset() {
    testTerrainScale=1;
    params_defaults();assert(params_set("DRVMODE",1,true)==0);
    fallPrevention.reset();autoClimb=AutoClimb();autoClimb.configure(false);
    descentGuard.reset();navigation.reset();yawControl.reset(0);
    pitchF=rateF=posM=velF=vCmd=xRef=zInt=uOut=0;
    cmdFwd=cmdSteer=encDiff=diffRateF=lookCmd=steerSteps=diffTarget=0;
    climbXRef=climbIntegral=climbDriveRef=climbRecovery=0;
    climbMode=stiffHold=driveReferenceActive=climbReferenceInitialized=false;
}
int main() {
    // Exact pre-fall snapshots from the supervised 2026-10-20 capture.
    // Full stick follows DRIVEV even above the informational free-speed estimate.
    const float captured[][4]={{1653867,.602f,3.91f,48.4f},{1654079,.633f,15.2f,130.9f},{1666810,.752f,6.68f,6.8f}};
    reset();
    assert(P.driveVEff==1 && P.driveVEff>P.vNoLoad && P.maxDriveV==1);
    cmdFwd=1;runController(.005f);
    assert(fallPrevention.operatorRef==1 && fallPrevention.effectiveRef==1);
    assert(fallPrevention.steeringScale==1 && !fallPrevention.recovering);
    pitchF=25;rateF=100;runController(.005f);
    assert(fallPrevention.effectiveRef==1 && fallPrevention.steeringScale==1);
    // Actual pre-fall frames: terrain feedback suppression erased/reversed
    // the forward catch even though the velocity demand was still full scale.
    reset();testTerrainScale=.55f;cmdFwd=1;velF=.608f;pitchF=5.85f;rateF=8.4f;
    runController(.005f);
    assert(appliedBalanceScale==1 && uOut>2.4f); // recorded old output: +0.03
    reset();testTerrainScale=.61f;cmdFwd=1;velF=.528f;pitchF=5.89f;rateF=10.7f;
    runController(.005f);
    assert(appliedBalanceScale==1 && uOut>2.1f); // recorded old output: -0.05
    for(const auto& sample:captured) {
        reset();cmdFwd=1;fallPrevention.operatorRef=P.driveVEff;velF=sample[1];pitchF=sample[2];rateF=sample[3];vCmd=.7253f;
        runController(.005f);
        assert(uOut>0 && vCmd>.7253f && std::fabs(requestedDriveV-P.driveVEff)<1e-5f);
    }
    reset();
    cmdFwd=1;fallPrevention.operatorRef=1;velF=.806f;vCmd=.725f;pitchF=15.2f;rateF=130.9f;
    runController(.005f);
    assert(std::fabs(requestedDriveV-P.driveVEff)<1e-5f && fallPrevention.operatorRef<=1);
    assert(uOut==P.aMax && vCmd>.725f && !satV);
    assert(.5f*(wheelTargetL+wheelTargetR)/P.stepsPerM>.725f);
    reset();
    cmdFwd=1;fallPrevention.operatorRef=P.driveVEff;velF=1.2f;vCmd=1.2f;
    runController(.005f);
    assert(vCmd>1.2f); // balance is still free to outrun the stick reference
    // Measured free speed remains informational; DRIVEV is unchanged.
    params_set("FREERPM",300,true);
    assert(std::fabs(P.driveVEff-1.0f)<1e-5f);
    params_set("FREERPM",0,true);
    // Autonomy obeys the same physical limit.
    reset();navigation.granted=true;navigation.velocity=1;
    runController(.005f);
    assert(std::fabs(requestedDriveV-P.driveVEff)<1e-5f);
    navigation.granted=false;
    reset();
    params_set("MAXSPD",P.countsPerM*.8f,true);
    cmdFwd=1;fallPrevention.operatorRef=.8f;velF=.9f;vCmd=.8f;pitchF=15;rateF=100;
    runController(.005f);
    assert(std::fabs(vCmd-.8f)<.00001f && uOut==P.aMax && satV);
    reset();
    cmdFwd=1;fallPrevention.operatorRef=1;pitchF=20;rateF=100;vCmd=.67f;
    runController(.005f);
    assert(satV && vCmd<=P.motorMa*.001f/P.hubKp+.00001f);
    // Velocity balancing does not inherit the bypassed current loop's I/Kp
    // virtual-state clamp. Both directions retain catch acceleration.
    for(float sign:{-1.0f,1.0f}) {
        reset();assert(params_set("DRVMODE",2)==0);
        cmdFwd=sign;pitchF=sign*20;rateF=sign*100;vCmd=sign*1.2f;
        runController(.005f);
        assert(sign*vCmd>1.2f && !satV && sign*uOut==P.aMax);
        vCmd=sign*(hub.velocityLimit()-.001f);
        runController(.005f);
        assert(satV && std::fabs(std::fabs(vCmd)-hub.velocityLimit())<.00001f);
    }
    puts("BLDC production controller: direct full-stick target, predictive governor bypass and balance anti-windup passed");
    return 0;
}
