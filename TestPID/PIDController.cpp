#include "PIDController.h"
#include "Saturation.h"
#include <iostream>

float PIDController::calculateVelocity(pose current_location, pose goal_location)
{
    std::cout << "\nCalculating Velocity\n";
    float current_error = pid_error.calculateCurrentError(current_location, goal_location);
    std::cout << "current_error: " << current_error << "\n";
    std::cout << "currentLocation: " << current_location.x << ", " << current_location.y << ", "  << current_location.theta << "\n";
    std::cout << "goalLocation: " << ", " << goal_location.x << ", " << goal_location.y << ", " << goal_location.theta << "\n";
    float error_derivative = pid_error.calculateDerivative(current_error);
    this->updateErrorIntegrator(goal_location,current_error);
    float error_integral = pid_error.getIntegrator();
    float desired_velocity = (gains.KP*current_error) + (gains.KI*error_integral) + (gains.KD*error_derivative);
    desired_velocity = Saturation::saturation(desired_velocity, MAX_VELOCITY);
    pid_error.setPriorError(current_error);
    return desired_velocity;
}

void PIDController::updateErrorIntegrator(pose goal_location, float current_error)
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
