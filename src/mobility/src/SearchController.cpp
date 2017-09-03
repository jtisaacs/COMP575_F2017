#include "SearchController.h"
//#include "Vec2D.hpp"

SearchController::SearchController()
{
    rng = new random_numbers::RandomNumberGenerator();
}

SearchController::SearchController(std::string rover_name)
{
    const float d = 0.5;
    const float OUTER_BOUNDARY = 7.5;
    //float waypoints_x1 [] = {OUTER_BOUNDARY-d, -(OUTER_BOUNDARY-d), -(OUTER_BOUNDARY-d),    OUTER_BOUNDARY-d,   OUTER_BOUNDARY-d, -(OUTER_BOUNDARY-2*d), -(OUTER_BOUNDARY-2*d),    OUTER_BOUNDARY-2*d, OUTER_BOUNDARY-2*d, -(OUTER_BOUNDARY-3*d), -(OUTER_BOUNDARY-3*d),    OUTER_BOUNDARY-3*d, OUTER_BOUNDARY-3*d, -(OUTER_BOUNDARY-4*d), -(OUTER_BOUNDARY-4*d),   OUTER_BOUNDARY-4*d, OUTER_BOUNDARY-4*d, OUTER_BOUNDARY-4*d, -(OUTER_BOUNDARY-5*d), -(OUTER_BOUNDARY-5*d),    OUTER_BOUNDARY-5*d,    OUTER_BOUNDARY-5*d};
    //float waypoints_y1 [] = {OUTER_BOUNDARY-d,    OUTER_BOUNDARY-d, -(OUTER_BOUNDARY-d), -(OUTER_BOUNDARY-d), OUTER_BOUNDARY-2*d,    OUTER_BOUNDARY-2*d, -(OUTER_BOUNDARY-2*d), -(OUTER_BOUNDARY-2*d), OUTER_BOUNDARY-3*d,    OUTER_BOUNDARY-3*d, -(OUTER_BOUNDARY-3*d), -(OUTER_BOUNDARY-3*d), OUTER_BOUNDARY-4*d,    OUTER_BOUNDARY-4*d, -(OUTER_BOUNDARY-4*d), -(OUTER_BOUNDARY-4*d), OUTER_BOUNDARY-5*d, OUTER_BOUNDARY-5*d,    OUTER_BOUNDARY-5*d, -(OUTER_BOUNDARY-5*d), -(OUTER_BOUNDARY-5*d),    OUTER_BOUNDARY-6*d};

    //float waypoints_x2 [] = {OUTER_BOUNDARY-5*d, -(OUTER_BOUNDARY-6*d), -(OUTER_BOUNDARY-6*d),    OUTER_BOUNDARY-6*d, OUTER_BOUNDARY-6*d, OUTER_BOUNDARY-6*d, -(OUTER_BOUNDARY-7*d), -(OUTER_BOUNDARY-7*d),    OUTER_BOUNDARY-7*d, OUTER_BOUNDARY-7*d, OUTER_BOUNDARY-7*d, -(OUTER_BOUNDARY-8*d), -(OUTER_BOUNDARY-8*d),    OUTER_BOUNDARY-8*d, OUTER_BOUNDARY-8*d, OUTER_BOUNDARY-8*d, -(OUTER_BOUNDARY-9*d), -(OUTER_BOUNDARY-9*d),   OUTER_BOUNDARY-9*d,   OUTER_BOUNDARY-9*d, OUTER_BOUNDARY-9*d, -(OUTER_BOUNDARY-10*d), -(OUTER_BOUNDARY-10*d),   OUTER_BOUNDARY-10*d,   OUTER_BOUNDARY-10*d, OUTER_BOUNDARY-10*d, -(OUTER_BOUNDARY-11*d), -(OUTER_BOUNDARY-11*d),    OUTER_BOUNDARY-11*d, OUTER_BOUNDARY-11*d, OUTER_BOUNDARY-11*d, -(OUTER_BOUNDARY-12*d), -(OUTER_BOUNDARY-12*d),    OUTER_BOUNDARY-12*d, OUTER_BOUNDARY-12*d};
    //float waypoints_y2 [] = {OUTER_BOUNDARY-6*d,    OUTER_BOUNDARY-6*d, -(OUTER_BOUNDARY-6*d), -(OUTER_BOUNDARY-6*d), OUTER_BOUNDARY-7*d, OUTER_BOUNDARY-7*d,    OUTER_BOUNDARY-7*d, -(OUTER_BOUNDARY-7*d), -(OUTER_BOUNDARY-7*d), OUTER_BOUNDARY-8*d, OUTER_BOUNDARY-8*d,    OUTER_BOUNDARY-8*d, -(OUTER_BOUNDARY-8*d), -(OUTER_BOUNDARY-8*d), OUTER_BOUNDARY-9*d, OUTER_BOUNDARY-9*d,    OUTER_BOUNDARY-9*d, -(OUTER_BOUNDARY-9*d), -(OUTER_BOUNDARY-9*d), OUTER_BOUNDARY-10*d, OUTER_BOUNDARY-10*d,    OUTER_BOUNDARY-10*d, -(OUTER_BOUNDARY-10*d), -(OUTER_BOUNDARY-10*d), OUTER_BOUNDARY-11*d, OUTER_BOUNDARY-11*d,    OUTER_BOUNDARY-11*d, -(OUTER_BOUNDARY-11*d), -(OUTER_BOUNDARY-11*d), OUTER_BOUNDARY-12*d, OUTER_BOUNDARY-12*d,    OUTER_BOUNDARY-12*d, -(OUTER_BOUNDARY-12*d), -(OUTER_BOUNDARY-12*d), OUTER_BOUNDARY-13*d};

    float waypoints_x1 [] = {0.000000, 0.500000, 0.500000, -0.500000, -0.500000, 2.000000, 2.000000, -2.000000, -2.000000, 3.500000, 3.500000, -3.500000, -3.500000, 5.000000, 5.000000, -5.000000, -5.000000, 6.500000, 6.500000, -6.500000, -6.500000,};
    float waypoints_y1 [] = {0.500000, 0.500000, -0.500000, -0.500000, 2.000000, 2.000000, -2.000000, -2.000000, 3.500000, 3.500000, -3.500000, -3.500000, 5.000000, 5.000000, -5.000000, -5.000000, 6.500000, 6.500000, -6.500000, -6.500000, 7.000000,};
    float waypoints_x2 [] = {0.000000, 1.000000, 1.000000, -1.000000, -1.000000, 2.500000, 2.500000, -2.500000, -2.500000, 4.000000, 4.000000, -4.000000, -4.000000, 5.500000, 5.500000, -5.500000, -5.500000, 7.000000, 7.000000, -7.000000, -7.000000,};
    float waypoints_y2 [] = {1.000000, 1.000000, -1.000000, -1.000000, 2.500000, 2.500000, -2.500000, -2.500000, 4.000000, 4.000000, -4.000000, -4.000000, 5.500000, 5.500000, -5.500000, -5.500000, 7.000000, 7.000000, -7.000000, -7.000000, 7.000000,};
    float waypoints_x3 [] = {0.000000, 1.500000, 1.500000, -1.500000, -1.500000, 3.000000, 3.000000, -3.000000, -3.000000, 4.500000, 4.500000, -4.500000, -4.500000, 6.000000, 6.000000, -6.000000, -6.000000,};
    float waypoints_y3 [] = {1.500000, 1.500000, -1.500000, -1.500000, 3.000000, 3.000000, -3.000000, -3.000000, 4.500000, 4.500000, -4.500000, -4.500000, 6.000000, 6.000000, -6.000000, -6.000000, 7.000000,};

    
    if (rover_name == "ajax")
    {
        int array_length = sizeof(waypoints_x1)/sizeof(waypoints_x1[0]);
        fillStack(waypoints_x1, waypoints_y1, array_length);
    }
    else if (rover_name == "aeneas")
    {
        int array_length = sizeof(waypoints_x2)/sizeof(waypoints_x2[0]);
        fillStack(waypoints_x2, waypoints_y2, array_length);
    }
    else if (rover_name == "achilles"
    {
        int array_length = sizeof(waypoints_x3)/sizeof(waypoints_x3[0]);
        fillStack(waypoints_x3, waypoints_y3, array_length);
    }             
}

void SearchController::fillStack(float waypoints_x [], float waypoints_y [], int array_length)
{
    pose next_position;
    for (int i = 0; i < array_length; i++)
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
    double new_theta = rng->gaussian(current_location.theta, 0.25);
    double new_radius = rng->uniformReal(0,2.0);
    next_waypoint.x = current_location.x + new_radius * cos(new_theta);
    next_waypoint.y = current_location.y + new_radius * sin(new_theta);
    return next_waypoint;
}
