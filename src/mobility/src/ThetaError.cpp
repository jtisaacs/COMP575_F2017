#include "ThetaError.h"
#include <angles/angles.h>

ThetaError::ThetaError()
{
    resetIntegrator();
    prior_error = 0.0;
}

float ThetaError::calculateCurrentError(float goal_theta, float current_theta)
{
    return angles::shortest_angular_distance(current_theta, goal_theta);
}

void ThetaError::resetIntegrator()
{
    integrator = 0.0;
}

void ThetaError::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float ThetaError::getIntegrator()
{
    return integrator;
}

float ThetaError::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void ThetaError::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}



