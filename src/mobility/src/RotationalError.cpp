#include "RotationalError.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>
RotationalError::RotationalError(): PIDError()
{

}

float RotationalError::calculateCurrentError(pose current_location, pose goal_location)
{
    goal_location.theta = atan2(goal_location.y - current_location.y, goal_location.x - current_location.x);
    float current_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);
    return current_error;
}


