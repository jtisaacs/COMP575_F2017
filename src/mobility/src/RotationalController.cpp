#include "RotationalController.h"

#include <math.h>
#include <angles/angles.h>

RotationalController::RotationalController()
{
    gains.KP = 0.75;
    gains.KI = 0.0;
    gains.KD = 0.0;
    pid_error = RotationalError();
}

bool RotationalController::checkForNewGoal(geometry_msgs::Pose2D goal_location)
{
    float change_in_goal_theta = abs(angles::shortest_angular_distance(goal_location_prior.theta, goal_location.theta));
    return change_in_goal_theta > FLOAT_COMPARISON_THRESHOLD;
}
