#ifndef THETAERROR_H
#define THETAERROR_H

class ThetaError
{

public:

    ThetaError();
    float calculateCurrentError(float goal_theta, float current_theta);
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

#endif // THETAERROR_H
