#include <cmath>
#include "TranslationalController.h"

TranslationalController::TranslationalController() : PIDController( pid_error )
{
    gains.KP = 1.5;
    gains.KI = 0.001;
    gains.KD = 0.0;
}

bool TranslationalController::checkForNewGoal(pose goal_location)
{
    float change_in_goal_location = hypot(goal_location.x-goal_location_prior.x, goal_location.y-goal_location_prior.y);
    return change_in_goal_location > FLOAT_COMPARISON_THRESHOLD;
}
