#ifndef TRANSLATIONALERROR_H
#define TRANSLATIONALERROR_H

#include "PIDError.h"
#include <cmath>

class TranslationalError : public PIDError
{

public:
    TranslationalError();
    float calculateCurrentError(pose currentLocation, pose goalLocation);
    // HTB
    void updateIntegrator(float current_error);
    void resetIntegrator();
    float getIntegrator();
    float calculateDerivative(float current_error);
    void setPriorError(float prior_error);

private:
    float prior_error;
    float integrator;
    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;

};

#endif // TRANSLATIONALERROR_H
