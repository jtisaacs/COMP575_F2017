#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "PIDError.h"
#include "Saturation.h"

class PIDController
{

public:
    PIDController( PIDError &p ) : pid_error(p) {}
    virtual float calculateVelocity(pose current_location, pose goal_location)=0;
    void updateErrorIntegrator(pose goal_location, float current_error);
    float getCurrentError();
    virtual bool isGoalChanged(pose goal_location)=0;
    virtual bool isGoalReached(pose current_location, pose goal_location)=0;
    float current_error;
private:


protected:
    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
    static const float MAX_VELOCITY = 0.3;
    struct gains_struct{
        float KP;
        float KI;
        float KD;
    };
    struct gains_struct gains;
    pose goal_location_prior;
    PIDError &pid_error;
};

#endif // PIDCONTROLLER_H
