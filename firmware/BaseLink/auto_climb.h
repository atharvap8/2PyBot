#pragma once
#include <cmath>
#include <stdint.h>

// Inferred sustained load, not a terrain-angle or temperature sensor.
// Fixed initial thresholds preserve the existing 107 saved parameters.
struct AutoClimb {
    enum Policy : uint8_t { AUTOMATIC=0, FORCE_NORMAL=1, FORCE_CLIMB=2, FORCE_STIFF=3 };
    enum State : uint8_t { NORMAL=0, SUSPECT=1, CLIMB=2, LIMITED=3, MANUAL=4, AUTO_DISABLED=5,
                           DOWNCLIMB=6, EXITING=7, COOLDOWN=8 };
    enum Kind : uint8_t { NO_LOAD=0, UP_LOAD=1, DOWN_LOAD=2 };
    enum Flag : uint16_t { INTENT=1, TRACKING_ERROR=2, PITCH_BIAS=4, DYNAMIC=8,
                           FOLLOWING_LIMIT=16, NO_PROGRESS=32, MANUAL_OVERRIDE=64, TRANSITION=128,
                           DESCENT_CUE=256, EXIT_EVIDENCE=512, REENTRY_HOLD=1024, OVERSPEED=2048 };
    bool enabled=true, active=false, seeded=false, actuatorLimited=false;
    Policy policy=AUTOMATIC;
    State state=NORMAL;
    uint16_t flags=0;
    Kind kind=NO_LOAD,candidateKind=NO_LOAD;
    uint8_t exitBlock=0; // load 1, following 2, dynamics 4, dwell 8, no exit cue 16
    float blend=0, referenceMean=0, velocityMean=0, pitchMean=0, rateMean=0;
    float followLeftMean=0,followRightMean=0,commandMean=0;
    float trackingError=0,pitchBias=0,followingError=0;
    float stableSeconds=0,suspectSeconds=0,flatSeconds=0,noProgressSeconds=0;
    float mismatchSeconds=0,followingGoodSeconds=0,activeSeconds=0,lastGoal=0;
    float forwardBaseline=2.0f,reverseBaseline=-2.0f;
    float neutralSeconds=0,cooldownSeconds=0,kindSeconds=0,lastOperator=0,motionDirection=1,overspeed=0;
    bool exitPending=false;
    float probeSeconds=0,probeBadSeconds=0,probeGoodSeconds=0,probeRetrySeconds=0,failedProbeBias=0;
    float referenceBlend=0;

    static float clip(float x,float lo,float hi) { return x<lo?lo:x>hi?hi:x; }
    void resetDynamics() {
        const bool wasEnabled=enabled;
        const Policy wasPolicy=policy;
        *this=AutoClimb();enabled=wasEnabled;policy=wasPolicy;
        state=policy!=AUTOMATIC?MANUAL:enabled?NORMAL:AUTO_DISABLED;
    }
    bool targetClimb() const { return policy==FORCE_CLIMB || (policy==AUTOMATIC && enabled && active); }
    void select(Policy next) {
        policy=next;active=false;kind=candidateKind=NO_LOAD;
        stableSeconds=suspectSeconds=flatSeconds=noProgressSeconds=cooldownSeconds=kindSeconds=0;
        exitPending=false;probeSeconds=probeBadSeconds=probeGoodSeconds=probeRetrySeconds=failedProbeBias=0;
        state=next==AUTOMATIC ? (enabled?NORMAL:AUTO_DISABLED) : MANUAL;
    }
    void configure(bool on) {
        enabled=on;select(AUTOMATIC);
    }
    void update(float goal,float velocity,float pitch,float rate,float steering,
                float leftCommand,float rightCommand,float leftVelocity,float rightVelocity,
                float normalFullSpeed,float hazard,bool airborne,bool suspended,bool accelCapped,float dt,
                float operatorRequest=NAN) {
        dt=clip(dt,0.0f,.05f);flags=0;exitBlock=0;
        (void)accelCapped; // individual acceleration clamps are not an exit veto
        probeRetrySeconds=std::fmax(0.0f,probeRetrySeconds-dt);
        if(!std::isfinite(operatorRequest))operatorRequest=goal;
        if(!std::isfinite(goal) || !std::isfinite(velocity) || !std::isfinite(pitch) ||
           !std::isfinite(rate) || !std::isfinite(leftCommand) || !std::isfinite(rightCommand) ||
           !std::isfinite(leftVelocity) || !std::isfinite(rightVelocity)) {
            stableSeconds=suspectSeconds=flatSeconds=0;flags|=DYNAMIC;
            return; // retain controller selection; invalid observations cannot trigger a switch
        }
        const float alpha=dt/(.40f+dt);
        if(!seeded) {
            referenceMean=goal;velocityMean=velocity;pitchMean=pitch;rateMean=rate;
            commandMean=std::fmax(std::fabs(leftCommand),std::fabs(rightCommand));
            followLeftMean=leftCommand-leftVelocity;followRightMean=rightCommand-rightVelocity;
            lastGoal=goal;lastOperator=operatorRequest;seeded=true;
        } else {
            referenceMean+=alpha*(goal-referenceMean);velocityMean+=alpha*(velocity-velocityMean);
            pitchMean+=alpha*(pitch-pitchMean);rateMean+=alpha*(rate-rateMean);
            commandMean+=alpha*(std::fmax(std::fabs(leftCommand),std::fabs(rightCommand))-commandMean);
            followLeftMean+=alpha*(leftCommand-leftVelocity-followLeftMean);
            followRightMean+=alpha*(rightCommand-rightVelocity-followRightMean);
        }
        trackingError=referenceMean-velocityMean;
        followingError=std::fmax(std::fabs(followLeftMean),std::fabs(followRightMean));
        const float followTolerance=std::fmax(.12f,.30f*commandMean);
        if(followingError>followTolerance) { mismatchSeconds+=dt;followingGoodSeconds=0; }
        else { mismatchSeconds=0;followingGoodSeconds+=dt; }
        if(mismatchSeconds>=.50f)actuatorLimited=true;
        if(followingGoodSeconds>=1.0f)actuatorLimited=false;
        if(actuatorLimited)flags|=FOLLOWING_LIMIT;

        // Gamepad LOW scales the 0.25 m/s climb request to 0.125 m/s.
        // Entry still needs 0.15 m/s; active exit/progress must accept LOW.
        const bool intent=std::fabs(operatorRequest)>=(active || blend>.01f?.06f:.15f);
        const bool directionChange=operatorRequest*lastOperator<0 && std::fabs(operatorRequest-lastOperator)>.08f;
        const bool change=std::fabs(operatorRequest-lastOperator)>.08f || directionChange;
        if(std::fabs(operatorRequest)>=.03f)motionDirection=operatorRequest>=0?1.0f:-1.0f;
        else if(!active && std::fabs(velocityMean)>.08f)motionDirection=velocityMean>=0?1.0f:-1.0f;
        neutralSeconds=std::fabs(operatorRequest)<.03f?neutralSeconds+dt:0;
        const bool dynamic=airborne || suspended || std::fabs(steering)>.25f ||
                           hazard>=.50f || std::fabs(pitch)>=12.0f || std::fabs(rate)>=120.0f;
        const bool exitDynamic=airborne || suspended || std::fabs(steering)>.65f ||
                               hazard>=.50f || std::fabs(pitch)>=12.0f || std::fabs(rate)>=120.0f;
        if(intent)flags|=INTENT;
        if(dynamic)flags|=DYNAMIC;
        if(change || !intent || dynamic)stableSeconds=0;
        else stableSeconds=std::fmin(10.0f,stableSeconds+dt);
        lastGoal=goal;lastOperator=operatorRequest;
        const float full=std::fmax(.05f,normalFullSpeed);
        const float normalised=clip(std::fabs(referenceMean)/full,0,1);
        const float baseline=referenceMean>=0?forwardBaseline:reverseBaseline;
        const float expectedPitch=baseline*normalised;
        pitchBias=pitchMean-expectedPitch;
        const float errorThreshold=std::fmax(.12f,.35f*std::fabs(referenceMean));
        const bool lag=std::fabs(trackingError)>errorThreshold;
        const bool severeLag=std::fabs(trackingError)>std::fmax(.18f,.65f*std::fabs(referenceMean));
        const bool strongBias=std::fabs(pitchBias)>=2.0f;
        const bool bias=strongBias || (severeLag && std::fabs(pitchBias)>=1.0f);
        if(lag)flags|=TRACKING_ERROR;
        if(bias)flags|=PITCH_BIAS;

        const float alignedBias=motionDirection*pitchBias;
        const float alignedVelocity=motionDirection*velocityMean;
        const float deficit=motionDirection*trackingError;
        overspeed=std::fmax(0.0f,alignedVelocity-std::fabs(referenceMean));
        const bool ahead=overspeed>std::fmax(.08f,.25f*std::fabs(referenceMean));
        if(ahead)flags|=OVERSPEED;
        const bool rolling=neutralSeconds>=1.5f && std::fabs(velocityMean)>.15f;
        const bool downCue=alignedVelocity>.08f &&
                           ((ahead && alignedBias<=-1.0f) ||
                            (alignedBias<=-2.5f && alignedVelocity>std::fmax(.10f,.65f*std::fabs(referenceMean))) ||
                            (rolling && alignedBias<=-1.0f));
        const bool upCue=deficit>errorThreshold &&
                         (alignedBias>=2.0f || (severeLag && alignedBias>=1.0f));
        candidateKind=downCue?DOWN_LOAD:upCue?UP_LOAD:NO_LOAD;
        if(downCue)flags|=DESCENT_CUE;
        if(cooldownSeconds>0 && blend<.01f)cooldownSeconds=std::fmax(0.0f,cooldownSeconds-dt);
        if(cooldownSeconds>0)flags|=REENTRY_HOLD;

        if(policy!=AUTOMATIC) {
            flags|=MANUAL_OVERRIDE;state=MANUAL;suspectSeconds=flatSeconds=0;
        } else if(!enabled) {
            state=AUTO_DISABLED;active=false;suspectSeconds=flatSeconds=0;
        } else if(active) {
            activeSeconds+=dt;
            // A short neutral pause on a slope retains compensation. An unsuccessful
            // attempt is flagged, not followed by a sudden gain change or motor cut.
            if(intent && std::fabs(velocityMean)<.025f && lag && !dynamic)noProgressSeconds+=dt;
            else noProgressSeconds=0;
            if(noProgressSeconds>=8.0f)flags|=NO_PROGRESS;
            if(candidateKind!=NO_LOAD && candidateKind!=kind && !dynamic && !exitPending)kindSeconds+=dt;
            else kindSeconds=0;
            if(kindSeconds>=.50f) { kind=candidateKind;kindSeconds=flatSeconds=0; }
            const bool downHeld=kind==DOWN_LOAD && (ahead || alignedBias<=-1.0f || rolling);
            const bool loadHeld=downHeld || (kind==UP_LOAD && alignedBias>=2.25f && intent);
            const bool trackingAcceptable=std::fabs(trackingError)<std::fmax(.12f,.35f*std::fabs(referenceMean));
            const bool improved=failedProbeBias<=0 || std::fabs(pitchBias)<std::fmax(.75f,failedProbeBias-.50f);
            const bool movingClear=intent && trackingAcceptable && std::fabs(rateMean)<15.0f &&
                                   ((!loadHeld && std::fabs(pitchBias)<2.25f) ||
                                    (kind==UP_LOAD && activeSeconds>=8.0f && improved && std::fabs(pitchMean)<8.0f));
            const bool stoppedClear=neutralSeconds>=4.0f && std::fabs(velocityMean)<.06f &&
                                    std::fabs(pitchMean)<1.25f && std::fabs(rateMean)<10.0f;
            const bool ready=(movingClear || stoppedClear) && !exitDynamic && !actuatorLimited && hazard<.30f &&
                             activeSeconds>=3.0f && blend>.99f && probeRetrySeconds<=0;
            if(loadHeld && !stoppedClear)exitBlock|=1;
            if(actuatorLimited)exitBlock|=2;
            if(exitDynamic || hazard>=.30f)exitBlock|=4;
            if(activeSeconds<3 || blend<.99f)exitBlock|=8;
            if(!movingClear && !stoppedClear)exitBlock|=16;
            // Exit is accumulated evidence, not a five-second perfect-sample
            // streak. Balance acceleration caps are never a flatness veto.
            if(ready && !exitPending) { flatSeconds+=dt;flags|=EXIT_EVIDENCE; }
            else if(exitDynamic || actuatorLimited || (loadHeld && !stoppedClear))flatSeconds=0;
            else if(!exitPending)flatSeconds=std::fmax(0.0f,flatSeconds-.50f*dt);
            if(!exitPending && flatSeconds>=(stoppedClear?1.0f:2.0f)) {
                exitPending=true;probeSeconds=probeBadSeconds=probeGoodSeconds=0;
            }
            if(exitPending) {
                probeSeconds+=dt;
                const bool tooFast=kind==DOWN_LOAD && overspeed>std::fmax(.04f,.15f*std::fabs(referenceMean));
                const bool badTracking=std::fabs(trackingError)>std::fmax(.12f,.40f*std::fabs(referenceMean));
                const bool hardAbort=exitDynamic || actuatorLimited || hazard>=.50f;
                if(probeSeconds>.75f && (tooFast || badTracking))probeBadSeconds+=dt;
                else probeBadSeconds=std::fmax(0.0f,probeBadSeconds-dt);
                if(hardAbort || probeBadSeconds>=.30f) {
                    exitPending=false;flatSeconds=0;probeRetrySeconds=6.0f;
                    failedProbeBias=!hardAbort && std::fabs(pitchBias)>=1.5f?std::fabs(pitchBias):0;
                } else {
                    const bool referenceSettled=kind==DOWN_LOAD || referenceBlend<.10f;
                    const bool normalHealthy=blend<.01f && referenceSettled && !tooFast && !badTracking && hazard<.30f &&
                                             std::fabs(rateMean)<15.0f;
                    probeGoodSeconds=normalHealthy?probeGoodSeconds+dt:0;
                    if(probeGoodSeconds>=.50f) {
                        active=false;exitPending=false;kind=NO_LOAD;cooldownSeconds=3.0f;
                        stableSeconds=suspectSeconds=flatSeconds=0;failedProbeBias=0;
                    }
                }
            }
            state=actuatorLimited || (flags&NO_PROGRESS) ? LIMITED : exitPending?EXITING:
                  active?(kind==DOWN_LOAD?DOWNCLIMB:CLIMB):EXITING;
        } else {
            activeSeconds=noProgressSeconds=flatSeconds=0;
            const bool eligibleMotion=(intent && stableSeconds>=1.0f) || rolling;
            // Back-driven overspeed may itself exceed the following band; allow
            // descent compensation while retaining LIMITED diagnostics.
            const bool candidate=eligibleMotion && !dynamic && (!actuatorLimited || downCue) &&
                                 std::fabs(rateMean)<12.0f && candidateKind!=NO_LOAD && blend<.01f &&
                                 (cooldownSeconds<=0 || (downCue && overspeed>.35f));
            suspectSeconds=candidate?suspectSeconds+dt:0;
            if(suspectSeconds>=(downCue?.75f:strongBias?1.25f:2.50f)) {
                active=true;kind=candidateKind;activeSeconds=0;suspectSeconds=0;
                state=kind==DOWN_LOAD?DOWNCLIMB:CLIMB;
            }
            else state=blend>.01f?EXITING:cooldownSeconds>0?COOLDOWN:actuatorLimited?LIMITED:candidate?SUSPECT:NORMAL;
            // Slowly learn expected pitch only during healthy, low-error normal
            // tracking; this is a heuristic baseline, not a measured ground angle.
            const bool learn=intent && stableSeconds>=1.0f && !dynamic && !actuatorLimited &&
                             !active && blend<.01f && hazard<.01f && normalised>.20f &&
                             std::fabs(trackingError)<std::fmax(.15f,.25f*std::fabs(referenceMean)) &&
                             std::fabs(rateMean)<8.0f && std::fabs(pitchMean)<6.0f;
            if(learn) {
                float &value=referenceMean>=0?forwardBaseline:reverseBaseline;
                value+=dt/(20.0f+dt)*(clip(pitchMean/normalised,-5.0f,5.0f)-value);
            }
        }
        const float blendTarget=exitPending?(probeSeconds<1.0f?.25f:0.0f):(targetClimb()?1.0f:0.0f);
        blend+=clip(blendTarget-blend,-dt/.75f,dt/.75f);
        blend=clip(blend,0,1);
        // Keep the descent limit throughout hand-back; uphill probation tests
        // normal control at the operator's requested reference gradually.
        if(active && exitPending && kind==DOWN_LOAD)referenceBlend=1;
        else if(targetClimb() && !exitPending)referenceBlend=std::fmax(referenceBlend,blend);
        else referenceBlend=std::fmax(blend,referenceBlend-dt/1.5f);
        referenceBlend=clip(referenceBlend,0,1);
        if(blend>.001f && blend<.999f)flags|=TRANSITION;
    }
};
