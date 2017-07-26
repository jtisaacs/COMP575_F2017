#include "SearchController.h"
//#include "Vec2D.hpp"

SearchController::SearchController()
{
    rng = new random_numbers::RandomNumberGenerator();
}

SearchController::SearchController(std::string rover_name)
{
    const float d = 0.5;
    float waypoints_x [] = {7.0-d, -(7.0-d), -(7.0-d),    7.0-d,   7.0-d, -(7.0-2*d), -(7.0-2*d),    7.0-2*d, 7.0-2*d, -(7.0-3*d), -(7.0-3*d),    7.0-3*d, 7.0-3*d, -(7.0-4*d), -(7.0-4*d),   7.0-4*d, 7.0-4*d, 7.0-4*d, -(7.0-5*d), -(7.0-5*d),    7.0-5*d,    7.0-5*d};
    float waypoints_y [] = {7.0-d,    7.0-d, -(7.0-d), -(7.0-d), 7.0-2*d,    7.0-2*d, -(7.0-2*d), -(7.0-2*d), 7.0-3*d,    7.0-3*d, -(7.0-3*d), -(7.0-3*d), 7.0-4*d,    7.0-4*d, -(7.0-4*d), -(7.0-4*d), 7.0-5*d, 7.0-5*d,    7.0-5*d, -(7.0-5*d), -(7.0-5*d),    7.0-6*d};

    float waypoints_x2 [] = {7.0-5*d, -(7.0-6*d), -(7.0-6*d),    7.0-6*d, 7.0-6*d, 7.0-6*d, -(7.0-7*d), -(7.0-7*d),    7.0-7*d, 7.0-7*d, 7.0-7*d, -(7.0-8*d), -(7.0-8*d),    7.0-8*d, 7.0-8*d, 7.0-8*d, -(7.0-9*d), -(7.0-9*d),   7.0-9*d,   7.0-9*d, 7.0-9*d, -(7.0-10*d), -(7.0-10*d),   7.0-10*d,   7.0-10*d, 7.0-10*d, -(7.0-11*d), -(7.0-11*d),    7.0-11*d, 7.0-11*d, 7.0-11*d, -(7.0-12*d), -(7.0-12*d),    7.0-12*d, 7.0-12*d};
    float waypoints_y2 [] = {7.0-6*d,    7.0-6*d, -(7.0-6*d), -(7.0-6*d), 7.0-7*d, 7.0-7*d,    7.0-7*d, -(7.0-7*d), -(7.0-7*d), 7.0-8*d, 7.0-8*d,    7.0-8*d, -(7.0-8*d), -(7.0-8*d), 7.0-9*d, 7.0-9*d,    7.0-9*d, -(7.0-9*d), -(7.0-9*d), 7.0-10*d, 7.0-10*d,    7.0-10*d, -(7.0-10*d), -(7.0-10*d), 7.0-11*d, 7.0-11*d,    7.0-11*d, -(7.0-11*d), -(7.0-11*d), 7.0-12*d, 7.0-12*d,    7.0-12*d, -(7.0-12*d), -(7.0-12*d), 7.0-13*d};

    if (rover_name == "ajax")
    {
        int array_length = sizeof(waypoints_x)/sizeof(waypoints_x[0]);
        fillStack(waypoints_x, waypoints_y, array_length);
    }
    else if (rover_name == "aeneas")
    {
        int array_length = sizeof(waypoints_x2)/sizeof(waypoints_x2[0]);
        fillStack(waypoints_x2, waypoints_y2, array_length);
    }
}

void SearchController::fillStack(float waypoints_x [], float waypoints_y [], int array_length)
{
    pose next_position;
    for (int i = array_length; i > 0; i--)
    {
        next_position.x = waypoints_x[i];
        next_position.y = waypoints_y[i];
        stack_waypoints.push(next_position);
    }
}

pose SearchController::getNextWaypoint(pose current_location)
{
    pose next_waypoint;
    if(stack_waypoints.empty())
    {
        next_waypoint = generateRandomWaypoint(current_location);
    }
    else
    {
        stack_waypoints.pop();
        next_waypoint = stack_waypoints.top();
    }
    return next_waypoint;
}

pose SearchController::getCurrentWaypoint(pose current_location)
{
    pose current_waypoint;
    if(stack_waypoints.empty())
    {
         current_waypoint = generateRandomWaypoint(current_location);
    }
    else
    {
        current_waypoint = stack_waypoints.top();
    }
    return current_waypoint;
}

bool SearchController::isSearchFinished()
{
    return stack_waypoints.empty();
}

pose SearchController::generateRandomWaypoint(pose current_location)
{
    pose next_waypoint;
    double newTheta = rng->gaussian(currentLocation.theta, 0.25);
    double newRadius = rng->uniformReal(0,2.0);
    next_waypoint.x = currentLocation.x + newRadius * cos(newTheta);
    next_waypoint.y = currentLocation.y + newRadius * sin(newTheta);
    return next_waypoint;
}
