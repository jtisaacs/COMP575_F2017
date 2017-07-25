#ifndef TRANSLATIONALCONTROLLER_H
#define TRANSLATIONALCONTROLLER_H

#include "TranslationalError.h"
#include "PIDController.h"


class TranslationalController : public PIDController
{

public:
    TranslationalController();
    bool isGoalChanged(pose goal_location);
    bool isGoalReached(pose current_location, pose goal_location);

private:
     TranslationalError pid_error;
};


#endif // TRANSLATIONALCONTROLLER_H
