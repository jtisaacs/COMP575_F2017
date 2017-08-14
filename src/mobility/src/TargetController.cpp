#include "TargetController.h"
using namespace std;

TargetController::TargetController(string rover_name)
{
    this->rover_name = rover_name;
    for (int i = 0; i < TOTAL_NUMBER_RESOURCES; i++) {
        targets.push_back(TargetState(i+1));
    }
    claimed_target_id = -1;
}

int TargetController::findIDClosestAvailableTarget()
{
    double min_distance = LONG_MAX;
    int id_closest_target = -1;
    vector<float> claimed_directions;
    for(int ii = 0; ii < TOTAL_NUMBER_RESOURCES; ii++)
    {
        if(targets[ii].isClaimed())
        {
            claimed_directions.push_back(atan2(targets[ii].getLocation().y, targets[ii].getLocation().x));
        }
    }
    for(int resource = 0; resource < TOTAL_NUMBER_RESOURCES; resource++)
    {
        if(targets[resource].isAvailable())
        {
            if (claimed_directions.size() > 0)
            {
                bool flag = true;
                for ( int itr = 0; itr < claimed_directions.size(); itr++ )
                {
                    if(fabs(claimed_directions[itr] - atan2(targets[resource].getLocation().y - current_location.y, targets[resource].getLocation().x - current_location.x)) < 0.25)
                    {
                        flag = false;
                    }
                }
                if(flag)
                {
                    double current_distance = computeDistanceBetweenWaypoints(targets[resource].getLocation(), current_location);
                    if(current_distance < min_distance)
                    {
                        min_distance = current_distance;
                        id_closest_target = resource;
                    }
                }
            }
            else
            {
                double current_distance = computeDistanceBetweenWaypoints(targets[resource].getLocation(), current_location);
                if(current_distance < min_distance)
                {
                    min_distance = current_distance;
                    id_closest_target = resource;
                }
            }
        }
    }
    return id_closest_target;
}
