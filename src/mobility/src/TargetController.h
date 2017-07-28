#ifndef TARGETCONTROLLER_H
#define TARGETCONTROLLER_H

class TargetController
{

public:
    TargetController();
    detectTargets();
private:
    vector<TargetState> targets;
};

#endif // TARGETCONTROLLER_H
