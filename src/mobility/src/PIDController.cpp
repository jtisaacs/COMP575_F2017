#include "PIDController.h"
#include "Saturation.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
ros::Publisher errorPublish;
using namespace std;

void PIDController::updateErrorIntegrator(pose goal_location, float current_error)
{
    if (this->isGoalChanged(goal_location))
    {
        pid_error.resetIntegrator();
    }
    else
    {
        pid_error.updateIntegrator(current_error);
    }
}
