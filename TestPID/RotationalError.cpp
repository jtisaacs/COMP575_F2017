#include "RotationalError.h"
#include <cmath>
#include <iostream>
RotationalError::RotationalError()
{

}

float RotationalError::calculateCurrentError(pose current_location, pose goal_location)
{
    std::cout << "In Rotational method.";
    float current_error = M_PI - std::fabs(std::fmod(std::fabs(current_location.theta - goal_location.theta), 2*M_PI) - M_PI);
    std::cout << current_error;
    std::cout << "\n";
    std::cout << goal_location.theta;
    std::cout << "\n";
    std::cout << current_location.theta;
    return current_error;
}


