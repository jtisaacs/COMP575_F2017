#ifndef PIDERROR_H
#define PIDERROR_H

#include "Pose.h"

class PIDError
{
public:
    PIDError();
    void updateIntegrator(float current_error);
    void resetIntegrator();
    float getIntegrator();
    float calculateDerivative(float current_error);
    void setPriorError(float prior_error);
    virtual float calculateCurrentError(pose currentLocation, pose goalLocation)=0;

protected:

private:

    float prior_error;
    float integrator;
    static const float FLOAT_COMPARISON_THRESHOLD = 1E-6;
};

#endif // PIDERROR_H
