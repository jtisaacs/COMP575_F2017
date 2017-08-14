#ifndef TARGETCONTROLLER_H
#define TARGETCONTROLLER_H
#include <cmath>
#include <string>

class TargetController
{

public:
    TargetController(string rover_name);
    detectTargets();
private:
    int claimed_target_id;
    string rover_name;
    vector<TargetState> targets;
    int findIDClosestAvailableTarget();
    const int TOTAL_NUMBER_RESOURCES = 256;
};

#endif // TARGETCONTROLLER_H
