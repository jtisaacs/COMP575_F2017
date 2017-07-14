#include "TranslationalError.h"

float TranslationalError::calculateCurrentError(pose currentLocation, pose goalLocation)
{
    "In Translational method.";
    return hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);;
}
