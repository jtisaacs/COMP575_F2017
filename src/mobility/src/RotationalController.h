#ifndef ROTATIONALCONTROLLER_H
#define ROTATIONALCONTROLLER_H

#include "RotationalError.h"
#include "PIDController.h"


class RotationalController : public PIDController
{

public:
    RotationalController();
    bool isGoalChanged(pose goal_location);
    bool isGoalReached(pose current_location, pose goal_location);

private:
    RotationalError pid_error;
};

#endif // ROTATIONALCONTROLLER_H
