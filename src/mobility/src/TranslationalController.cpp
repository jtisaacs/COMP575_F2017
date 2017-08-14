#include <cmath>
#include "TranslationalController.h"

TranslationalController::TranslationalController()
{
    gains.KP = 1.5;
    gains.KI = 0.001;
    gains.KD = 0.0;
    resetIntegrator();
    prior_error = 0.0;
}

bool TranslationalController::isGoalChanged(pose goal_location)
{
    float change_in_goal_location = hypotf(goal_location.x-goal_location_prior.x, goal_location.y-goal_location_prior.y);
    return change_in_goal_location > FLOAT_COMPARISON_THRESHOLD;
}

bool TranslationalController::isGoalReached(pose current_location, pose goal_location)
{
    float distance_to_goal = hypotf(goal_location.x-current_location.x, goal_location.y-current_location.y);
    return distance_to_goal < 0.5; // 0.5 meter circle around target waypoint.
}

// Horrible Things Below
float TranslationalController::calculateVelocity(pose current_location, pose goal_location)
{
    current_error = calculateCurrentError(current_location, goal_location);
    float error_derivative = calculateDerivative(current_error);
    this->updateErrorIntegrator(goal_location,current_error);
    float error_integral = getIntegrator();
    float desired_velocity = (gains.KP*current_error) + (gains.KI*error_integral) + (gains.KD*error_derivative);
    desired_velocity = Saturation::saturation(desired_velocity, MAX_VELOCITY);
    setPriorError(current_error);
    return desired_velocity;
}

void TranslationalController::updateErrorIntegrator(pose goal_location, float current_error)
{
    if (this->isGoalChanged(goal_location))
    {
        resetIntegrator();
    }
    else
    {
        updateIntegrator(current_error);
    }
}

float TranslationalController::calculateCurrentError(pose currentLocation, pose goalLocation)
{
    return hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);
}

void TranslationalController::resetIntegrator()
{
    integrator = 0.0;
}

void TranslationalController::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float TranslationalController::getIntegrator()
{
    return integrator;
}

float TranslationalController::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void TranslationalController::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}

