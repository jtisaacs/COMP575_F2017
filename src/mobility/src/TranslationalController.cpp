#include <math.h>
#include "TranslationalController.h"

TranslationalController::TranslationalController()
{
    gains.KP = 1.5;
    gains.KI = 0.001;
    gains.KD = 0.0;
    pid_error = TranslationalError();
}

bool TranslationalController::checkForNewGoal(geometry_msgs::Pose2D goal_location)
{
    float change_in_goal_location = hypot(goal_location.x-goal_location_prior.x, goal_location.y-goal_location_prior.y);
    return change_in_goal_location > FLOAT_COMPARISON_THRESHOLD;
}
