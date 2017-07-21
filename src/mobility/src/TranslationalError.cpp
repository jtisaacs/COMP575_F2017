#include "TranslationalError.h"

TranslationalError::TranslationalError()
{

}
float TranslationalError::calculateCurrentError(pose currentLocation, pose goalLocation)
{
    return hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);
}
