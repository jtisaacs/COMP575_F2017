#ifndef SEARCHCONTROLLER_H
#define SEARCHCONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <string>
#include <stack>          // std::stack
#include "Pose.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

  public:

    SearchController();
    SearchController(std::string rover_name);

    void fillStack(float waypoints_x [], float waypoints_y [], int array_length);
    pose search(pose current_location);
    pose getNextWaypoint(pose current_location);
    pose getCurrentWaypoint(pose current_location);
    bool isSearchFinished();
    pose generateRandomWaypoint(pose current_location);

  private:

    random_numbers::RandomNumberGenerator* rng;
    std::stack<pose> stack_waypoints;
};

#endif // SEARCHCONTROLLER_H
