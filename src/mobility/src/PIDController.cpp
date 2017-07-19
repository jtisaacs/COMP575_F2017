#include "PIDController.h"
#include "Saturation.h"

#include <math.h>
#include <angles/angles.h>

float PIDController::calculateVelocity(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    float current_error = pid_error.calculateCurrentError(currentLocation, goalLocation);
    float error_derivative = pid_error.calculateDerivative(current_error);
    this->updateErrorIntegrator(goal_location,current_error);
    float error_integral = pid_error.getIntegrator();
    float desired_velocity = (gains.KP*current_error) + (gains.KI*error_integral) + (gains.KD*error_derivative);
    desired_velocity = Saturation::saturation(desired_velocity, MAX_VELOCITY);
    pid_error.setPriorError(current_error);
    return desired_velocity;
}

void PIDController::updateErrorIntegrator(geometry_msgs::Pose2D goal_location, float current_error)
{
    if (checkForNewGoal(goal_location))
    {
        pid_error.resetIntegrator();
    }
    else
    {
        pid_error.updateIntegrator(current_error);
    }
}
