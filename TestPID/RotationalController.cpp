#include "RotationalController.h"

#include <cmath>
#include <iostream>

RotationalController::RotationalController() : PIDController( this->pid_error )
{
    gains.KP = 0.75;
    gains.KI = 0.0;
    gains.KD = 0.0;
    pid_error = RotationalError();
}

bool RotationalController::checkForNewGoal(pose goal_location)
{
    float change_in_goal_theta =  M_PI - std::fabs(std::fmod(std::fabs(goal_location_prior.theta - goal_location.theta), 2*M_PI) - M_PI);
    std::cout << "\nChecking for new goal.\n";
    std::cout << change_in_goal_theta << ", " << goal_location.theta << ", " << goal_location_prior.theta;
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}
