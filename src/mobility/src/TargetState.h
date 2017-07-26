#ifndef TARGETSTATE_H
#define TARGETSTATE_H

#include "Pose.h"

class TargetState
{

public:
    TargetState(int identifier);
    pose getLocation();
    State getState();

private:
    int identifier;
    pose location;
    enum State
    {
        Initial,
        Detected,
        Claimed,
        PickedUp,
        DroppedOff
    };
    State state;

};

#endif // TARGETSTATE_H
