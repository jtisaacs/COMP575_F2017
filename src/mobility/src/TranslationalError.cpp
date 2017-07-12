#include "TranslationalError.h"

float RotationalError::calculateCurrentError(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    return hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);;
}
