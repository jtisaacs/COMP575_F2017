#include "RotationalError.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>
RotationalError::RotationalError()
{

}

float RotationalError::calculateCurrentError(pose current_location, pose goal_location)
{
    float current_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);
    return current_error;
}


