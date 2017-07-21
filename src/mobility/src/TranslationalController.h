#ifndef TRANSLATIONALCONTROLLER_H
#define TRANSLATIONALCONTROLLER_H

#include "TranslationalError.h"
#include "PIDController.h"


class TranslationalController : public PIDController
{

public:
    TranslationalController();
    TranslationalError pid_error;
    bool checkForNewGoal(pose goal_location);

private:

};


#endif // TRANSLATIONALCONTROLLER_H
