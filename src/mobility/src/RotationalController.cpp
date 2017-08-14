#include "RotationalController.h"

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>

RotationalController::RotationalController()
{
    gains.KP = 1.25;
    gains.KI = 0.0;
    gains.KD = 0.0;
    resetIntegrator();
    prior_error = 0.0;
}

bool RotationalController::isGoalChanged(pose goal_location)
{
    float change_in_goal_theta =  fabs(angles::shortest_angular_distance(goal_location_prior.theta, goal_location.theta));
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}

bool RotationalController::isGoalReached(pose current_location, pose goal_location)
{
    float distance_to_goal = fabs(angles::shortest_angular_distance(current_location.theta, goal_location.theta));
    return distance_to_goal < 0.25; // 0.25 radians around goal heading
}
// Horrible Things Below
float RotationalController::calculateVelocity(pose current_location, pose goal_location)
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

void RotationalController::updateErrorIntegrator(pose goal_location, float current_error)
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


float RotationalController::calculateCurrentError(pose current_location, pose goal_location)
{
    goal_location.theta = atan2(goal_location.y - current_location.y, goal_location.x - current_location.x);
    float current_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);
    return current_error;
}

// HTB
void RotationalController::resetIntegrator()
{
    integrator = 0.0;
}

void RotationalController::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float RotationalController::getIntegrator()
{
    return integrator;
}

float RotationalController::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void RotationalController::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}

float RotationalController::getCurrentError()
{
    return current_error;
}

