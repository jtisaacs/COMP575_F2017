#ifndef TARGETSTATE_H
#define TARGETSTATE_H

#include "Pose.h"

class TargetState
{

public:
    TargetState(int identifier);
    pose getLocation();
    bool isInitial();
    bool isDetected();
    bool isClaimed();
    bool isPickedUp();
    bool isDroppedOff();
    bool isAvailable();
    void detect(pose location);
    void claim();
    void pickUp();
    void dropOff();
    void giveUp();
    int getState();

private:
    int identifier;
    pose location;
    pose *location_pointer;
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
