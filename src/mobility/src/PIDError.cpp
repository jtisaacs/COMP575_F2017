#include "PIDError.h"
#include <iostream>

PIDError::PIDError()
{
    resetIntegrator();
    prior_error = 0.0;
}

void PIDError::resetIntegrator()
{
    integrator = 0.0;
}

void PIDError::updateIntegrator(float current_error)
{
    integrator = integrator + current_error;
}

float PIDError::getIntegrator()
{
    return integrator;
}

float PIDError::calculateDerivative(float current_error)
{
    return prior_error - current_error;
}

void PIDError::setPriorError(float prior_error)
{
    this->prior_error = prior_error;
}
