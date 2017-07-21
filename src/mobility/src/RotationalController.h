#ifndef ROTATIONALCONTROLLER_H
#define ROTATIONALCONTROLLER_H

#include "RotationalError.h"
#include "PIDController.h"


class RotationalController : public PIDController
{

public:
    RotationalController();
    RotationalError pid_error;
    bool checkForNewGoal(pose goal_location);

private:

};

#endif // ROTATIONALCONTROLLER_H
