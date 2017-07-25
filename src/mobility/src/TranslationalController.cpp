#include <cmath>
#include "TranslationalController.h"

TranslationalController::TranslationalController() : PIDController( this->pid_error )
{
    gains.KP = 1.5;
    gains.KI = 0.001;
    gains.KD = 0.0;
    pid_error = TranslationalError();
}

bool TranslationalController::isGoalChanged(pose goal_location)
{
    float change_in_goal_location = hypotf(goal_location.x-goal_location_prior.x, goal_location.y-goal_location_prior.y);
    return change_in_goal_location > FLOAT_COMPARISON_THRESHOLD;
}

bool TranslationalController::isGoalReached(pose current_location, pose goal_location)
{
    float distance_to_goal = hypotf(goal_location.x-current_location.x, goal_location.y-current_location.y);
    return distance_to_goal < 0.5; // 0.5 meter circle around target waypoint.
}
