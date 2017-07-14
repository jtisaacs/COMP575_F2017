#ifndef TRANSLATIONALERROR_H
#define TRANSLATIONALERROR_H

#include "PIDError.h"
#include <cmath>

class TranslationalError : public PIDError
{

public:

    float calculateCurrentError(pose currentLocation, pose goalLocation);

private:

};

#endif // TRANSLATIONALERROR_H
