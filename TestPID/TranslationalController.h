#ifndef TRANSLATIONALCONTROLLER_H
#define TRANSLATIONALCONTROLLER_H

#include "TranslationalError.h"
#include "PIDController.h"


class TranslationalController : public PIDController
{

public:

    TranslationalController();

private:

    bool checkForNewGoal(pose goalLocation);
    TranslationalError pid_error;
};


#endif // TRANSLATIONALCONTROLLER_H
