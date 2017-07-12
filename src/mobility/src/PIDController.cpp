#include "PIDController.h"
#include <math.h>
#include <angles/angles.h>

PIDController::PIDController()
{
//    resetDistanceErrorIntegrator();
    distanceErrorPrior = 0.0;
    theta_error_prior = 0.0;
    goal_theta_prior = 0.0;
}

float PIDController::calculateTranslationalVelocity(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
//    if(hypot(goalLocation.x-goalLocationPrior.x, goalLocation.y-goalLocationPrior.y) > FLOAT_COMPARISON_THRESHOLD)
//    {
//        // We have a new goal, so we should reset the integrator.
//        resetDistanceErrorIntegrator();
//    }
    float distanceError = hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);
    distanceErrorIntegrator = distanceErrorIntegrator + distanceError;
    float distanceErrorDerivative = (distanceError - distanceErrorPrior);
    float desiredTranslationalVelocity = (KP_TRANSLATIONAL*distanceError) + (KI_TRANSLATIONAL*distanceErrorIntegrator) + (KD_TRANSLATIONAL*distanceErrorDerivative);
    distanceErrorPrior = distanceError;

    if(desiredTranslationalVelocity > MAX_LINEAR_VELOCITY)
    {
        desiredTranslationalVelocity = MAX_LINEAR_VELOCITY;
    }
    else if (desiredTranslationalVelocity < -MAX_LINEAR_VELOCITY)
    {
        desiredTranslationalVelocity = -MAX_LINEAR_VELOCITY;
    }
    return desiredTranslationalVelocity;
}

float PIDController::calculateRotationalVelocity(float current_theta, float goal_theta)
{
    float current_error = theta_error.calculateCurrentError(goal_theta, current_theta);
    updateThetaErrorIntegrator(goal_theta, current_error);
    float error_integral = theta_error.getIntegrator();
    float error_derivative = theta_error.calculateDerivative(current_error);
    float desiredAngularVelocity = (KP_ROTATIONAL*current_error) + (KI_ROTATIONAL*error_integral) + (KD_ROTATIONAL*error_derivative);
    desiredAngularVelocity = saturation(desiredAngularVelocity,MAX_ANGULAR_VELOCITY);

    theta_error.setPriorError(current_error);
    goal_theta_prior = goal_theta;

    return desiredAngularVelocity;
}

float PIDController::saturation(float independent_variable, float maximum_absolute_value)
{
    independent_variable = applyMaximumLimit(independent_variable,maximum_absolute_value);
    independent_variable = applyMinimumLimit(independent_variable,-maximum_absolute_value);
    return independent_variable;
}

float PIDController::applyMaximumLimit(float independent_variable, float maximum_value_limit)
{
    return independent_variable > maximum_value_limit ? maximum_value_limit : independent_variable;
}

float PIDController::applyMinimumLimit(float independent_variable, float minimum_value_limit)
{
    return independent_variable < minimum_value_limit ? minimum_value_limit : independent_variable;
}


bool PIDController::checkForNewGoalTheta(float goal_theta_prior, float goal_theta)
{
    float change_in_goal_theta = abs(angles::shortest_angular_distance(goal_theta_prior, goal_theta));
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}

void PIDController::updateThetaErrorIntegrator(float goal_theta, float current_error)
{
    if (checkForNewGoalTheta(goal_theta_prior, goal_theta))
    {
        theta_error.resetIntegrator();
    }
    else
    {
        theta_error.updateIntegrator(current_error);
    }
}
