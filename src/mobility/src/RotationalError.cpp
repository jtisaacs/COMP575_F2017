#include "RotationalError.h"
#include <angles/angles.h>

float RotationalError::calculateCurrentError(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    return angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
}


