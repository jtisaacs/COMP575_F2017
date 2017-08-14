#ifndef TRANSLATIONALCONTROLLER_H
#define TRANSLATIONALCONTROLLER_H

#include "TranslationalError.h"
#include "PIDController.h"


class TranslationalController
{

public:
    TranslationalController();
    float calculateVelocity(pose current_location, pose goal_location);
    bool isGoalChanged(pose goal_location);
    bool isGoalReached(pose current_location, pose goal_location);
    // Horrible Things Below
    void updateErrorIntegrator(pose goal_location, float current_error);
    float getCurrentError();
    float current_error;


private:
     // Horrible Things Below
     static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
     static const float MAX_VELOCITY = 0.3;
     struct gains_struct{
         float KP;
         float KI;
         float KD;
     };
     struct gains_struct gains;
     pose goal_location_prior;
     // Error
     float calculateCurrentError(pose currentLocation, pose goalLocation);
     // HTB
     void updateIntegrator(float current_error);
     void resetIntegrator();
     float getIntegrator();
     float calculateDerivative(float current_error);
     void setPriorError(float prior_error);
     // Error
     float prior_error;
     float integrator;
};


#endif // TRANSLATIONALCONTROLLER_H
