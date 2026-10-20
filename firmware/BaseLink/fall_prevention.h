#pragma once
#include <cmath>
#include <stdint.h>

// Reference governor, not a replacement for LQR and not a wheel-velocity stop.
// Operator-reference shaping is separate from physical balance acceleration.
struct FallPrevention {
    enum Reason : uint8_t { SHAPING=1, REVERSAL=2, RECOVERY=4, SPEED_MARGIN=8, AIRBORNE=16, YAW_LIMIT=32 };
    float operatorRef = 0.0f;
    float effectiveRef = 0.0f;
    float refAcceleration = 0.0f;
    float risk = 0.0f;
    float hazardRisk = 0.0f; // instantaneous hazard, excludes mild recovery latch
    float predictedTilt = 0.0f;
    float steeringScale = 1.0f;
    bool recovering = false;
    bool fullBalanceFeedback = false;
    uint8_t reasons = 0;
    float calmSeconds = 0.0f;

    static float clip(float x, float lo, float hi) { return x<lo?lo:x>hi?hi:x; }
    static float score(float value, float warn, float critical) { return clip((value-warn)/(critical-warn),0.0f,1.0f); }
    void reset() { *this = FallPrevention(); }

    // Diagnostic/unrestricted mode: direct operator target, no slew, risk-based
    // reference cancellation or predictive steering reduction. Hard stops live
    // in the main state machine and motor transport, independently of this.
    void directReference(float requested,float pitchDegrees,float rateDps) {
        reset();operatorRef=effectiveRef=requested;
        predictedTilt=std::fmax(std::fabs(pitchDegrees),std::fabs(pitchDegrees+.08f*rateDps));
        fullBalanceFeedback=std::fabs(pitchDegrees)>=6 || std::fabs(rateDps)>=40;
    }

    void update(float requested, float measuredVelocity, float pitchDegrees, float rateDps,
                float commonCommandVelocity, float maximumVelocity, float maximumAcceleration,
                float maximumTilt, float dt, bool airborne) {
        dt = clip(dt,0.0f,.05f);
        reasons = 0;
        const float maxVelocity = std::fmax(.05f,maximumVelocity);
        const bool speedLimited = maximumVelocity > 0;
        const float goal = speedLimited ? clip(requested,-maxVelocity,maxVelocity) : requested;
        const float referenceAccelerationLimit = std::fmin(2.40f,.70f*std::fmax(.05f,maximumAcceleration));
        const float referenceJerkLimit = 18.0f;
        float error = goal-operatorRef;
        // Stop increasing a virtual reference away from a newly reversed target.
        // This resets reference acceleration only; physical balance acceleration is untouched.
        if (refAcceleration*error<0.0f) refAcceleration=0.0f;
        float wantedAcceleration = dt>0.0f ? clip(error/dt,-referenceAccelerationLimit,referenceAccelerationLimit) : 0.0f;
        refAcceleration += clip(wantedAcceleration-refAcceleration,-referenceJerkLimit*dt,referenceJerkLimit*dt);
        const float next = operatorRef+refAcceleration*dt;
        if (error*(goal-next)<=0.0f) { operatorRef=goal;refAcceleration=0.0f; }
        else operatorRef=speedLimited ? clip(next,-maxVelocity,maxVelocity) : next;

        const bool reversal = goal*measuredVelocity<-.015f && std::fabs(measuredVelocity)>.12f;
        const bool shaping = std::fabs(goal-operatorRef)>.035f || std::fabs(refAcceleration)>.10f;
        if(shaping) reasons|=SHAPING;
        if(reversal) reasons|=REVERSAL;
        predictedTilt = std::fmax(std::fabs(pitchDegrees),std::fabs(pitchDegrees+.08f*rateDps));
        const float warn = std::fmin(18.0f,.45f*maximumTilt);
        const float critical = std::fmin(28.0f,.75f*maximumTilt);
        const float angleRisk = score(predictedTilt,warn,std::fmax(warn+1.0f,critical));
        const float commandRatio = speedLimited ? std::fmax(std::fabs(commonCommandVelocity),std::fabs(measuredVelocity))/maxVelocity : 0;
        const float speedRisk = speedLimited ? score(commandRatio,.94f,.995f) : 0;
        if(speedRisk>0.0f) reasons|=SPEED_MARGIN;
        hazardRisk = airborne?1.0f:std::fmax(angleRisk,speedRisk);
        const float instantaneousRisk = hazardRisk;
        if(instantaneousRisk>0.0f || airborne) { recovering=true;calmSeconds=0.0f; }
        else if(recovering) {
            const bool calm = std::fabs(pitchDegrees)<8.0f && std::fabs(rateDps)<60.0f && commandRatio<.92f;
            calmSeconds = calm ? calmSeconds+dt : 0.0f;
            if(calmSeconds>=.10f) { recovering=false;calmSeconds=0.0f; }
        }
        risk = recovering ? std::fmax(.08f,instantaneousRisk) : 0.0f;
        if(airborne) { risk=1.0f;reasons|=AIRBORNE;operatorRef=0.0f;refAcceleration=0.0f; }
        if(recovering) reasons|=RECOVERY;
        // Temporarily cancel operator velocity-tracking error as risk grows,
        // letting the unchanged pitch/rate feedback catch the body.
        const float intervention=airborne?1.0f:hazardRisk*hazardRisk;
        effectiveRef = operatorRef+intervention*(measuredVelocity-operatorRef);
        steeringScale = 1.0f-.65f*intervention;
        fullBalanceFeedback = !airborne && (shaping || reversal || recovering ||
                               std::fabs(pitchDegrees)>=6.0f || std::fabs(rateDps)>=40.0f);
    }

    void constrainForwardReference(float maximumOperatorRef, float measuredVelocity) {
        if(operatorRef>maximumOperatorRef) {
            operatorRef=maximumOperatorRef;refAcceleration=0.0f;reasons|=SHAPING;
            effectiveRef=operatorRef+hazardRisk*hazardRisk*(measuredVelocity-operatorRef);
            if(!(reasons&AIRBORNE)) fullBalanceFeedback=true;
        }
    }

    float allocateSteering(float commonSteps, float requestedSteeringSteps, float maximumSteps) {
        float headroom=maximumSteps>0 ? std::fmax(0.0f,maximumSteps-std::fabs(commonSteps)) : std::fabs(requestedSteeringSteps);
        float requested=requestedSteeringSteps*steeringScale;
        float applied=clip(requested,-headroom,headroom);
        if(std::fabs(applied-requestedSteeringSteps)>.5f) reasons|=YAW_LIMIT;
        return applied;
    }

    // Mild latched recovery must not wash out climb's slope compensation.
    float climbRecoveryScale() const {
        return (reasons&AIRBORNE)?1.0f:score(hazardRisk,.55f,.95f);
    }
};
