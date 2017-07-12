#ifndef ROTATIONALCONTROLLER_H
#define ROTATIONALCONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include "RotationalError.h"
#include "PIDController.h"


class RotationalController : public PIDController
{

public:

    RotationalController();

private:

    bool checkForNewGoal(geometry_msgs::Pose2D goalLocation);
};

#endif // ROTATIONALCONTROLLER_H
