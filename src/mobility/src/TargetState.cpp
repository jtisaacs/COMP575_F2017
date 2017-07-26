#include "TargetState.h"

class TargetState
{

    TargetState::TargetState(int identifier)
    {
        this->identifier = identifier;
        location = NULL;
        state = Initial;
    }

    pose getLocation()
    {
        return location;
    }

    bool isInitial() {
        return state == Initial;
    }

    bool isDetected() {
        return state == Detected;
    }

    bool isClaimed() {
        return state == Claimed;
    }

    bool isPickedUp() {
        return state == PickedUp;
    }

    bool isDroppedOff() {
        return state == DroppedOff;
    }

    void detect(pose location) {
        if (isInitial()) {
            this->location = location;
            state = Detected;
        }
    }

    void claim() {
        if (isDetected()) {
            state = Claimed;
        }
    }

    void pickUp() {
        if (isClaimed()) {
            state = PickedUp;
        }
    }

    void dropOff() {
        if (isPickedUp()) {
            state = DroppedOff;
        }
    }

    void giveUp() {
        if (isClaimed()) {
            state = Detected;
        }
    }

    State getState()
    {
        return state;
    }

};
