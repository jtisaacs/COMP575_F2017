#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include "ThetaError.h"

/**
 * This class implements a PID controller for the rovers. The code
 * here should not be modified.
 */
class PIDController
{

public:

    PIDController();
    float calculateTranslationalVelocity(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation);
    void resetDistanceErrorIntegrator();
    float calculateRotationalVelocity(float currentTheta, float goalTheta);
    void resetThetaErrorIntegrator();
    bool checkForNewGoalTheta(float goal_theta_prior, float goal_theta);
    void updateThetaErrorIntegrator(float goal_theta, float current_error);


private:
    float saturation(float independent_variable, float maximum_absolute_value);
    float applyMaximumLimit(float independent_variable, float maximum_value_limit);
    float applyMinimumLimit(float independent_variable, float minimum_value_limit);
    float distanceErrorIntegrator;
    float distanceErrorPrior;
    geometry_msgs::Pose2D goalLocationPrior;
    ThetaError theta_error;

    float thetaErrorIntegrator;
    float theta_error_prior;
    float goal_theta_prior;

    static const float KP_TRANSLATIONAL = 1.5;
    static const float KI_TRANSLATIONAL = 0.001;
    static const float KD_TRANSLATIONAL = 0.0;

    static const float KP_ROTATIONAL = 0.75;
    static const float KI_ROTATIONAL = 0.0;
    static const float KD_ROTATIONAL = 0.0;

    static const float MAX_LINEAR_VELOCITY = 0.3;
    static const float MAX_ANGULAR_VELOCITY = 0.3;
    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
};

#endif // PIDCONTROLLER_H
