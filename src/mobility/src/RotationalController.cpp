#include "RotationalController.h"

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <angles/angles.h>

RotationalController::RotationalController() : PIDController( this->pid_error )
{
    gains.KP = 0.75;
    gains.KI = 0.0;
    gains.KD = 0.0;
    pid_error = RotationalError();
}

bool RotationalController::checkForNewGoal(pose goal_location)
{
    float change_in_goal_theta =  abs(angles::shortest_angular_distance(goal_location_prior.theta, goal_location.theta));
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}
