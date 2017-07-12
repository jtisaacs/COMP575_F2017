#ifndef TRANSLATIONALCONTROLLER_H
#define TRANSLATIONALCONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include "TranslationalError.h"
#include "PIDController.h"


class TranslationalController : public PIDController
{

public:

    TranslationalController();

private:

    bool checkForNewGoal(geometry_msgs::Pose2D goalLocation);
};


#endif // TRANSLATIONALCONTROLLER_H
