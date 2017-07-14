#ifndef ROTATIONALERROR_H
#define ROTATIONALERROR_H

#include "PIDError.h"

class RotationalError : public PIDError
{

public:
    RotationalError();
    float calculateCurrentError(pose currentLocation, pose goalLocation);

private:

};

#endif // ROTATIONALERROR_H
