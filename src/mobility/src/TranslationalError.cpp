#include "TranslationalError.h"

TranslationalError::TranslationalError()
{
    resetIntegrator();
    prior_error = 0.0;
}
float TranslationalError::calculateCurrentError(pose currentLocation, pose goalLocation)
{
    return hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);
}

void TranslationalError::resetIntegrator()
{
    integrator = 0.0;
}

void TranslationalError::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float TranslationalError::getIntegrator()
{
    return integrator;
}

float TranslationalError::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void TranslationalError::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}
