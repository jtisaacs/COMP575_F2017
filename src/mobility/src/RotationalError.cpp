#include "RotationalError.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>
RotationalError::RotationalError()
{
    resetIntegrator();
    prior_error = 0.0;
}

float RotationalError::calculateCurrentError(pose current_location, pose goal_location)
{
    goal_location.theta = atan2(goal_location.y - current_location.y, goal_location.x - current_location.x);
    float current_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);
    return current_error;
}

// HTB
void RotationalError::resetIntegrator()
{
    integrator = 0.0;
}

void RotationalError::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float RotationalError::getIntegrator()
{
    return integrator;
}

float RotationalError::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void RotationalError::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}


