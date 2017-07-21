#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "PIDError.h"
#include "Saturation.h"

class PIDController
{

public:
    PIDController( PIDError &p ) : pid_error(p) {}
    float calculateVelocity(pose current_location, pose goal_location);
    float getCurrentError();
    virtual bool checkForNewGoal(pose goal_location){}

private:

    void updateErrorIntegrator(pose goal_location, float current_error);


protected:

    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
    static const float MAX_VELOCITY = 0.2;
    struct gains_struct{
        float KP;
        float KI;
        float KD;
    };
    struct gains_struct gains;
    pose goal_location_prior;
    PIDError &pid_error;
    float current_error;

};

#endif // PIDCONTROLLER_H
