#include <cstddef>
#include <cmath>
#include <string>
#include <sstream>
#include "TargetState.h"

TargetState::TargetState(int identifier)
{
    this->identifier = identifier;
    location_pointer = &location;
    location_pointer = NULL;
    state = Initial;
}

pose TargetState::getLocation()
{
    return location;
}

bool TargetState::isAvailable() {
    return isDetected() && !isDroppedOff() && !isClaimed() && !isPickedUp();
}

bool TargetState::isInitial() {
    return state == Initial;
}

bool TargetState::isDetected() {
    return state == Detected;
}

bool TargetState::isClaimed() {
    return state == Claimed;
}

bool TargetState::isPickedUp() {
    return state == PickedUp;
}

bool TargetState::isDroppedOff() {
    return state == DroppedOff;
}

void TargetState::detect(pose location) {
    if (isInitial()) {
        this->location.x = location.x;
        this->location.y = location.y;
        this->location.theta = location.theta;
        state = Detected;
    }
}

void TargetState::claim() {
    if (isDetected()) {
        state = Claimed;
    }
}

void TargetState::pickUp() {
    if (isClaimed() || isDetected()) {
        state = PickedUp;
    }
}

void TargetState::dropOff() {
    if (isPickedUp()) {
        state = DroppedOff;
    }
}

void TargetState::giveUp() {
    if (isClaimed()) {
        state = Detected;
    }
}

int TargetState::getState() {
    return state;
}
