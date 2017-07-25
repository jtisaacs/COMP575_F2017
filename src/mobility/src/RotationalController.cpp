#include "RotationalController.h"

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>

RotationalController::RotationalController() : pid_error(RotationalError()), PIDController( this->pid_error )
{
    gains.KP = 0.75;
    gains.KI = 0.0;
    gains.KD = 0.0;
}

bool RotationalController::isGoalChanged(pose goal_location)
{
    float change_in_goal_theta =  fabs(angles::shortest_angular_distance(goal_location_prior.theta, goal_location.theta));
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}

bool RotationalController::isGoalReached(pose current_location, pose goal_location)
{
    float distance_to_goal = fabs(angles::shortest_angular_distance(current_location.theta, goal_location.theta));
    return distance_to_goal < 0.25; // 0.25 radians around goal heading
}
