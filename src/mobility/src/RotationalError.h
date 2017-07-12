#ifndef ROTATIONALERROR_H
#define ROTATIONALERROR_H

#include "PIDError.h"

class RotationalError : public PIDError
{

public:

    float calculateCurrentError(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation);

private:

};

#endif // ROTATIONALERROR_H
