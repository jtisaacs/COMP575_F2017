#ifndef ROTATIONALCONTROLLER_H
#define ROTATIONALCONTROLLER_H

#include "RotationalError.h"
#include "PIDController.h"


class RotationalController : public PIDController
{

public:
    RotationalController();
    RotationalError pid_error;
private:


    bool checkForNewGoal(pose goalLocation);
};

#endif // ROTATIONALCONTROLLER_H
