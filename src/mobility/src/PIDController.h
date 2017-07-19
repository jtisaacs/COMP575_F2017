#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include "PIDError.h"
#include "Saturation.h"

class PIDController
{

public:

    PIDController( PIDError &p ) : pid_error(p) {}
    float calculateVelocity(geometry_msgs::Pose2D current_location, geometry_msgs::Pose2D goal_location);

private:

    void updateErrorIntegrator(geometry_msgs::Pose2D goal_location, float current_error);
    virtual bool checkForNewGoal(geometry_msgs::Pose2D goal_location){}

protected:

    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
    static const float MAX_VELOCITY = 0.3;
    struct gains_struct{
        float KP;
        float KI;
        float KD;
    };
    struct gains_struct gains;
    geometry_msgs::Pose2D goal_location_prior;
    PIDError &pid_error;
};

#endif // PIDCONTROLLER_H
