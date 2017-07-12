#ifndef TRANSLATIONALERROR_H
#define TRANSLATIONALERROR_H

#include "PIDError.h"
#include <math.h>

class TranslationalError : public PIDError
{

public:

    float calculateCurrentError(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation);

private:

};

#endif // TRANSLATIONALERROR_H
