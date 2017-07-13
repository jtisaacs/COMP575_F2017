#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "TranslationalController.h"
#include "RotationalController.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <string>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

TranslationalController translational_controller;
RotationalController rotational_controller;

string roverName;
char host[128];
bool publishedName = false;

int simulationMode = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;

geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;

bool avoiding_obstacle = false;
vector <geometry_msgs::Pose2D> savedPositions;
vector <geometry_msgs::Pose2D> cluster_detect;

std_msgs::Int16 targetClaimed; // ID of the claimed target
std_msgs::Int16 claimedTargetDetected; //ID of the claimed target once it has been detected by gatherer.

bool targetsCollected[256];     // array of booleans indicating whether each target ID has been found and delivered;  not used at this time.
bool targetsDetected[256];      // array of booleans indicating whether each target ID has been found
bool targetsAvailable[256];     // array of booleans indicating whether each target ID has not been claimed by a collector.
bool targetsAvailDetected[256]; // array of booleans indicating whether each target ID has been found and has not been claimed by a collector.
bool targetsHome[256]; // array of booleans indicating wheter each target ID has been delivered to home base.
geometry_msgs::Pose2D targetPositions[256];

// achilles random search
const int RANDOM_SIZE = 3;
const double MAX_LINEAR_VELOCITY = 0.2;
const double MAX_ANGULAR_VELOCITY = 0.3;

// ajax and aeneas waypoints, lawnmower search
const int LAWNMOWER_SIZE_1 = 22; //29JTI must change this to match size of waypoints_x.  Why??
const int LAWNMOWER_SIZE_2 = 35;

const float d = 0.5;
float waypoints_x [] = {7.5-d, -(7.5-d), -(7.5-d),    7.5-d,   7.5-d, -(7.5-2*d), -(7.5-2*d),    7.5-2*d, 7.5-2*d, -(7.5-3*d), -(7.5-3*d),    7.5-3*d, 7.5-3*d, -(7.5-4*d), -(7.5-4*d),   7.5-4*d, 7.5-4*d, 7.5-4*d, -(7.5-5*d), -(7.5-5*d),    7.5-5*d,    7.5-5*d};
float waypoints_y [] = {7.5-d,    7.5-d, -(7.5-d), -(7.5-d), 7.5-2*d,    7.5-2*d, -(7.5-2*d), -(7.5-2*d), 7.5-3*d,    7.5-3*d, -(7.5-3*d), -(7.5-3*d), 7.5-4*d,    7.5-4*d, -(7.5-4*d), -(7.5-4*d), 7.5-5*d, 7.5-5*d,    7.5-5*d, -(7.5-5*d), -(7.5-5*d),    7.5-6*d};

float waypoints_x2 [] = {7.5-5*d, -(7.5-6*d), -(7.5-6*d),    7.5-6*d, 7.5-6*d, 7.5-6*d, -(7.5-7*d), -(7.5-7*d),    7.5-7*d, 7.5-7*d, 7.5-7*d, -(7.5-8*d), -(7.5-8*d),    7.5-8*d, 7.5-8*d, 7.5-8*d, -(7.5-9*d), -(7.5-9*d),   7.5-9*d,   7.5-9*d, 7.5-9*d, -(7.5-10*d), -(7.5-10*d),   7.5-10*d,   7.5-10*d, 7.5-10*d, -(7.5-11*d), -(7.5-11*d),    7.5-11*d, 7.5-11*d, 7.5-11*d, -(7.5-12*d), -(7.5-12*d),    7.5-12*d, 7.5-12*d};
float waypoints_y2 [] = {7.5-6*d,    7.5-6*d, -(7.5-6*d), -(7.5-6*d), 7.5-7*d, 7.5-7*d,    7.5-7*d, -(7.5-7*d), -(7.5-7*d), 7.5-8*d, 7.5-8*d,    7.5-8*d, -(7.5-8*d), -(7.5-8*d), 7.5-9*d, 7.5-9*d,    7.5-9*d, -(7.5-9*d), -(7.5-9*d), 7.5-10*d, 7.5-10*d,    7.5-10*d, -(7.5-10*d), -(7.5-10*d), 7.5-11*d, 7.5-11*d,    7.5-11*d, -(7.5-11*d), -(7.5-11*d), 7.5-12*d, 7.5-12*d,    7.5-12*d, -(7.5-12*d), -(7.5-12*d), 7.5-13*d};

vector <geometry_msgs::Pose2D> achilles_waypoints;
vector <geometry_msgs::Pose2D> ajax_waypoints;
vector <geometry_msgs::Pose2D> aeneas_waypoints;

// ajax and aeneas cluster detection
int targets_detected_screenshot = 0;

int transitionsToAuto = 0;
double timeStampTransitionToAuto = 0.0;

// sorted queue
vector <geometry_msgs::Pose2D> queue;

// state machine states
#define STATE_MACHINE_INITIALIZE 0
#define STATE_MACHINE_ROTATE    1
#define STATE_MACHINE_TRANSLATE 2
#define STATE_MACHINE_POP_WAYPOINT 3
#define STATE_MACHINE_SCORE_TARGET 4
#define STATE_MACHINE_CLAIM_TARGET 5
#define STATE_MACHINE_RETURN_HOME 6
#define STATE_MACHINE_EXPLORE_NEARBY 7
#define STATE_MACHINE_CHANGE_MODE 8
#define STATE_MACHINE_RANDOM_SEARCH 9
#define STATE_MACHINE_EXPLORE_NEAR_ORIGIN 12
int stateMachineState = STATE_MACHINE_INITIALIZE;

#define MODE_SEARCHER 0 // mode variable for rovers which will be used to perform lawnmower search.
#define MODE_COLLECTOR 1 // mode variable for rovers which will be used to perform random search and collect tags found by lawnmower searcher.
int roverCurrentMode = MODE_SEARCHER; // initiall set all rovers to be collectors

#define CAPACITY_EMPTY 0 // indicates that rover has not claimed and is not carrying april tag
#define CAPACITY_CLAIMED 1 // indicates that rover has claimed an april tag, but hasn't found it yet.
#define CAPACITY_CARRYING 2 // inidcates that rover hs claimed an april tag, and is returning it to home base.
int roverCapacity = CAPACITY_EMPTY;

//Publishers
ros::Publisher velocity_publisher;
ros::Publisher state_publisher;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher target_pick_up_publisher;
ros::Publisher target_drop_off_publisher;

ros::Publisher message_publisher;

//Subscribers
ros::Subscriber joy_subscriber;
ros::Subscriber mode_subscriber;
ros::Subscriber target_subscriber;
ros::Subscriber obstacle_subscriber;
ros::Subscriber odometry_subscriber;
ros::Subscriber targets_collected_subscriber;
ros::Subscriber message_subscriber;

//Timers
ros::Timer state_machine_timer;
ros::Timer publish_status_timer;
ros::Timer kill_switch_timer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); // 

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr &message);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);

//Utility functions
double computeGoalTheta(geometry_msgs::Pose2D goalLocation, geometry_msgs::Pose2D currentLocation);
double computeDistanceToGoal(geometry_msgs::Pose2D goalLocation, geometry_msgs::Pose2D currentLocation);
void setGoalLocation(double goalLocationX, double goalLocationY);
void saveGoalPosition();
void packageMessage(string message_content);

int searchQueue();
void visitRandomLocation();
void claimResource(int resouceID);
void unclaimResource(int resouceID);
void homeResource(int resouceID);

bool cluster = false;

void reportDetected(std_msgs::Int16 msg);

void detectedMessage(vector<string> msg_parts);
void claimMessage(vector<string> msg_parts);
void unclaimMessage(vector<string> msg_parts);
void homeMessage(vector<string> msg_parts);

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    targetClaimed.data = -1; // initialize target claimed
    claimedTargetDetected.data = -1; // intially the gatherer has no claimed targets so -1 indicates that the gatherer is either looking for a claimed target or there is no current claimed target.

    if (argc >= 2)
    {
        roverName = argv[1];
        cout << "Welcome to the world of tomorrow " << roverName << "!  Mobility module started." << endl;
    } else
    {
        roverName = hostName;
        cout << "No Name Selected. Default is: " << roverName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (roverName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joy_subscriber = mNH.subscribe((roverName + "/joystick"), 10, joyCmdHandler);
    mode_subscriber = mNH.subscribe((roverName + "/mode"), 1, modeHandler);
    target_subscriber = mNH.subscribe((roverName + "/targets"), 10, targetHandler);
    obstacle_subscriber = mNH.subscribe((roverName + "/obstacle"), 10, obstacleHandler);
    odometry_subscriber = mNH.subscribe((roverName + "/odom/ekf"), 10, odometryHandler);
    targets_collected_subscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
    message_subscriber = mNH.subscribe(("messages"), 10, messageHandler);

    status_publisher = mNH.advertise<std_msgs::String>((roverName + "/status"), 1, true);
    velocity_publisher = mNH.advertise<geometry_msgs::Twist>((roverName + "/velocity"), 10);
    state_publisher = mNH.advertise<std_msgs::String>((roverName + "/state_machine"), 1, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    message_publisher = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_pick_up_publisher = mNH.advertise<sensor_msgs::Image>((roverName + "/targetPickUpImage"), 1, true);
    target_drop_off_publisher = mNH.advertise<sensor_msgs::Image>((roverName + "/targetDropOffImage"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    kill_switch_timer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    state_machine_timer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);

    for(int i = 0; i < 256; i++)
    {
        targetsAvailable[i] = true;  // initially all targets are available to be claimed although none have been detected yet.
    }

    ros::spin();

    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String stateMachineMsg;

    if ((simulationMode == 2 || simulationMode == 3)) // Robot is in automode
    {
        if (transitionsToAuto == 0)
        {
            // This is the firt time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitionsToAuto++;
            timeStampTransitionToAuto = ros::Time::now().toSec();
        }
        switch (stateMachineState)
        {
        case STATE_MACHINE_INITIALIZE:
        {
            stateMachineMsg.data = "INITIALIZING";
            if (roverName == "ajax" || roverName == "aeneas") // set mode to searcher
            {
                roverCurrentMode = MODE_SEARCHER; // set ajax and aeneas to be the lawnmower path searchers.

                if (roverName == "ajax") // assign waypoints to perform lawnmower
                {
                    setGoalLocation(waypoints_x[0],waypoints_y[0]);
                    geometry_msgs::Pose2D nextPosition;
                    for (int i = LAWNMOWER_SIZE_1; i > 0; i--)
                    {
                        nextPosition.x = waypoints_x[i];
                        nextPosition.y = waypoints_y[i];
                        ajax_waypoints.push_back(nextPosition);
                    }
                }
                else if (roverName == "aeneas") // assign waypoints to perform lawnmower
                {
                    setGoalLocation(waypoints_x2[0],waypoints_y2[0]);
                    geometry_msgs::Pose2D nextPosition;
                    for (int i = LAWNMOWER_SIZE_2; i > 0; i--)
                    {
                        nextPosition.x = waypoints_x2[i];
                        nextPosition.y = waypoints_y2[i];
                        aeneas_waypoints.push_back(nextPosition);
                    }
                }
            }
            else // all other robots, assign to collect
            {
                roverCurrentMode = MODE_COLLECTOR;
                setGoalLocation(currentLocation.x,currentLocation.y);
            }
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_ROTATE:
        {
            double rotational_velocity = 0.0;
            double randVel = rng->uniformReal(-.02, 0.02);
            float is_turned_towards_goal = abs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.25;
            if (is_turned_towards_goal)
            {
                rotational_velocity = rotational_controller.calculateVelocity(currentLocation,goalLocation);
                setVelocity(randVel, rotational_velocity); //add a little bit of random linear-velocity to make turns less sticky
            }
            else
            {
                rotational_velocity = 0.0;
                setVelocity(0.0, rotational_velocity); // stop
                stateMachineState = STATE_MACHINE_TRANSLATE; // move to translate step
            }
            std::stringstream converter;
            converter << angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) << ", " << rotational_velocity;

            stateMachineMsg.data = "ROTATING";
            break;
        }
        case STATE_MACHINE_TRANSLATE:
        {

            double rotational_velocity = 0.0;
            double translational_velocity = 0.0;


            double distanceToGoal = computeDistanceToGoal(goalLocation, currentLocation);
            if (distanceToGoal >= 0.25) // if you are not within 25cm of goal
            {
                rotational_velocity = rotational_controller.calculateVelocity(currentLocation,goalLocation);
                translational_velocity = translational_controller.calculateVelocity(currentLocation,goalLocation);
                setVelocity(translational_velocity, rotational_velocity);
            }
            else
            {
                rotational_velocity = 0.0;
                translational_velocity = 0.0;
                setVelocity(translational_velocity, rotational_velocity);

                // This is where the magic happens.  We must decide what to do once we have reached the desired location.
                if ( avoiding_obstacle )
                {
                    // this is part of assigning new goal, but it is taking into consideration, obstacles
                    avoiding_obstacle = false;

                    while (savedPositions.size() > 1)
                    {
                        savedPositions.pop_back();
                    }

                    goalLocation.x = savedPositions.back().x;
                    goalLocation.y = savedPositions.back().y;
                    goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
                    //                    pivot();
                    savedPositions.pop_back();
                    stateMachineState = STATE_MACHINE_ROTATE;
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (roverName == "ajax") && (ajax_waypoints.size() > 0) )
                {
                    stateMachineState = STATE_MACHINE_POP_WAYPOINT; // We are ajax and we are currently searching with remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (roverName == "aeneas") && (aeneas_waypoints.size() > 0) )
                {
                    stateMachineState = STATE_MACHINE_POP_WAYPOINT; // We are aeneas and we are currently searching with remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (roverName == "ajax") && (ajax_waypoints.size() <= 0) )
                {
                    stateMachineState = STATE_MACHINE_CHANGE_MODE; // We are ajax and we are currently searching with no remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (roverName == "aeneas") && (aeneas_waypoints.size() <= 0) )
                {
                    stateMachineState = STATE_MACHINE_CHANGE_MODE; // We are aeneas and we are currently searching with no remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_CLAIMED) )
                {
                    stateMachineState = STATE_MACHINE_EXPLORE_NEARBY; // We have gone to where we thought the target should be but didn't find it there.
                }
                else if ( (roverCurrentMode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_EMPTY) )
                {
                    stateMachineState = STATE_MACHINE_CHANGE_MODE;
                }
                else if ( (roverCurrentMode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_CARRYING) )
                {
                    stateMachineState = STATE_MACHINE_EXPLORE_NEAR_ORIGIN; // We have gone to where we thought the origin should be but didn't find it there.
                }
            }
            std::stringstream converter;
            converter <<
                         "G_T: " << goalLocation.theta << " C_T: " << currentLocation.theta << " A_V: " << rotational_velocity << " Dist: " << distanceToGoal << " L_V: " << translational_velocity << " G_X: " << goalLocation.x << " G_Y: " << goalLocation.y;

            stateMachineMsg.data = "TRANSLATING";//, " + converter.str();
            break;
        }
        case STATE_MACHINE_POP_WAYPOINT:
        {
            stateMachineMsg.data = "POPPING WAYPOINT";
            if (roverName == "ajax")
            {
                goalLocation.x = ajax_waypoints.back().x;
                goalLocation.y = ajax_waypoints.back().y;
                goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
                ajax_waypoints.pop_back();
            }
            if (roverName == "aeneas")
            {
                goalLocation.x = aeneas_waypoints.back().x;
                goalLocation.y = aeneas_waypoints.back().y;
                goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
                aeneas_waypoints.pop_back();
            }
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_CHANGE_MODE:
        {
            stateMachineMsg.data = "CHANGING MODE";
            roverCurrentMode = MODE_COLLECTOR; // change mode from lawnmower searcher to collector.
            stateMachineState = STATE_MACHINE_CLAIM_TARGET;
            break;
        }
        case STATE_MACHINE_EXPLORE_NEARBY:
        {
            stateMachineMsg.data = "EXPORING NEARBY";
            if (targetsHome[targetClaimed.data])
            {
                // This means that someone else has taken this target home alread, so something went wrong.  Drop it and claim a new target.
                stateMachineState = STATE_MACHINE_CLAIM_TARGET;
            }
            else
            {
                // It must be around here somewhere.  Keep searching for it.
                double r = rng->uniformReal(0.0, 1.0);
                double t = rng->uniformReal(0, 2 * M_PI);
                goalLocation.x = targetPositions[targetClaimed.data].x + r * cos(t); // Explore in a 1m circle around where you think the target is located.
                goalLocation.y = targetPositions[targetClaimed.data].y + r * sin(t);
                goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
                stateMachineState = STATE_MACHINE_ROTATE;
            }
            break;
        }
        case STATE_MACHINE_RETURN_HOME:
        {
            stateMachineMsg.data = "RETURNING TO HOME";
            // set angle to center as goal heading
            goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
            // set center as goal position
            goalLocation.x = 0.0;
            goalLocation.y = 0.0;
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
            // Comment
        }
        case STATE_MACHINE_CLAIM_TARGET:
        {
            stateMachineMsg.data = "CLAIMING TARGET";
            int result = searchQueue(); // search targets detected and avialble
            if(result != -1)
            {
                claimResource(result); // update targets detected and available array
                targetClaimed.data = result; // This should be where targetClaimed gets set.
                roverCapacity=CAPACITY_CLAIMED; // Update status of rover
                goalLocation.x = targetPositions[result].x;
                goalLocation.y = targetPositions[result].y;
                goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
                stateMachineState = STATE_MACHINE_ROTATE;
            }
            else
            {
                stateMachineState = STATE_MACHINE_RANDOM_SEARCH;
            }
            break;
        }
        case STATE_MACHINE_RANDOM_SEARCH:
        {
            stateMachineMsg.data = "RANDOMLY SEARCHING";
            visitRandomLocation();
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_EXPLORE_NEAR_ORIGIN:
        {
            stateMachineMsg.data = "EXPORING NEAR ORIGIN";
            double r = rng->uniformReal(0.0, 1);
            double t = rng->uniformReal(0, 2 * M_PI);
            goalLocation.x = 0.0 + r * cos(t); // Explore in a 1m circle around where you think the target is located.
            goalLocation.y = 0.0 + r * sin(t);
            goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
        }
        default:
        {
            stateMachineMsg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to see
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulationMode;

        stateMachineMsg.data = "WAITING, " + converter.str();
    }
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
    // the rover's kill switch wont be called.
    kill_switch_timer.stop();
    kill_switch_timer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocity_publisher.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message)
{

    for(int i = 0; i < message->tags.data.size(); i++)
    {
        //check if target has not yet been collected
        if ( message->tags.data[i] == 256 || !targetsHome[message->tags.data[i]] )
        {
            // We have encountered a target that is either the home location (256) or is a target that we haven't dropped off yet.
            if ( roverCurrentMode==MODE_COLLECTOR && (message->tags.data[i] == 256) && roverCapacity==CAPACITY_CARRYING )
            {
                // We are a collector, we are carrying a target, and we reached home base
                // This is actually the STATE_MACHINE_SCORE_TARGET state, but it now lives inside the target handler.

                setVelocity(0.0,0.0); // stop the rover
                homeResource(targetClaimed.data); // deliver the id of the target that you were carrying.
                //publish to scoring code
                target_drop_off_publisher.publish(message->image); // publish the image that you are dropping off so it can be scored.

                roverCapacity=CAPACITY_EMPTY; // set the capacity of this rover back to empty

                targetClaimed.data = -1; // we no longer have a claimed target so we need to claim a new target.

                stateMachineState = STATE_MACHINE_CLAIM_TARGET;
            }
            else if (!(message->tags.data[i] == 256) && roverCurrentMode==MODE_COLLECTOR && (message->tags.data[i]==targetClaimed.data) && roverCapacity==CAPACITY_CLAIMED)
            {
                // We have found the claimed target, so we need to pick it up and return to home.
                setVelocity(0.0,0.0); // stop the rover
                roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying

                // copy target ID to class variable
                claimedTargetDetected.data = message->tags.data[i];
                // publish to scoring code
                target_pick_up_publisher.publish(message->image); //publish the image that you are picking up.
                // publish detected target
                target_collected_publisher.publish(claimedTargetDetected);
                stateMachineState = STATE_MACHINE_RETURN_HOME;
            }
            else if  (!(message->tags.data[i] == 256) && roverCurrentMode==MODE_COLLECTOR && (message->tags.data[i]!=targetClaimed.data) && roverCapacity==CAPACITY_CLAIMED)
            {
                // we have found a target that isn't the one we claimed.  We need to unclaim the one we have and claim a new one.
                if (targetsAvailDetected[message->tags.data[i]])
                {
                    // here we need to unclaim our original and claim this one.
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying
                    claimResource(message->tags.data[i]); // update targets detected and available array
                    unclaimResource(targetClaimed.data);


                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.


                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    target_pick_up_publisher.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    target_collected_publisher.publish(claimedTargetDetected);
                    stateMachineState = STATE_MACHINE_RETURN_HOME;
                }
                else if ( !targetsDetected[message->tags.data[i]] )
                {
                    // here we need to unclaim our original, then reportDectected and claim this one.
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying
                    claimResource(message->tags.data[i]); // update targets detected and available array
                    unclaimResource(targetClaimed.data);
                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.

                    std_msgs::Int16 tag;
                    tag.data = message->tags.data[i];
                    reportDetected(tag);

                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    target_pick_up_publisher.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    target_collected_publisher.publish(claimedTargetDetected);
                    stateMachineState = STATE_MACHINE_RETURN_HOME;
                }
            }
            else if (!(message->tags.data[i] == 256) && roverCurrentMode==MODE_COLLECTOR && roverCapacity==CAPACITY_EMPTY) // we have found a target
            {
                if (targetsAvailDetected[message->tags.data[i]])
                {
                    // here we need to claim this one.
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying


                    claimResource(message->tags.data[i]); // update targets detected and available array
                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.


                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    target_pick_up_publisher.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    target_collected_publisher.publish(claimedTargetDetected);

                    stateMachineState = STATE_MACHINE_RETURN_HOME;
                }
                else if ( !targetsDetected[message->tags.data[i]] )
                {
                    // here we need to reportDectected and claim this one.
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying

                    std_msgs::Int16 tag;
                    tag.data = message->tags.data[i];
                    reportDetected(tag);


                    claimResource(message->tags.data[i]); // update targets detected and available array
                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.


                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    target_pick_up_publisher.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    target_collected_publisher.publish(claimedTargetDetected);
                    stateMachineState = STATE_MACHINE_RETURN_HOME;
                }
            }
            else if ( !targetsDetected[message->tags.data[i]] ) //we have encountered a tag that has not been detected yet and we are in Search Mode.  Report it.
            {
                std_msgs::Int16 tag;
                tag.data = message->tags.data[i];
                reportDetected(tag);
            }
        }
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulationMode = message->data;
    setVelocity(0.0, 0.0);
}
void saveGoalPosition()
{
    geometry_msgs::Pose2D savedPosition;
    savedPosition.x = goalLocation.x;
    savedPosition.y = goalLocation.y;
    savedPosition.theta = goalLocation.theta;
    savedPositions.push_back(savedPosition);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( (message->data > 0) && roverCurrentMode==MODE_COLLECTOR ) //!(avoiding_obstacle)
    {
        if ( (roverCapacity==CAPACITY_CARRYING) && (angles::shortest_angular_distance(currentLocation.theta, atan2(-currentLocation.y, -currentLocation.x)) < M_PI_2) )
        {
            setVelocity(-0.2,0.0);
            // we are going to the goal with a target.  Just wait till the other guy moves out of our way.

            saveGoalPosition();
            //try not to move.
            goalLocation.x = currentLocation.x;
            goalLocation.y = currentLocation.y;
            goalLocation.theta = currentLocation.theta;


        }
        else
        {
            setVelocity(-0.2,0.0);
            // we are not delivering a target so lets get out of the way.
            saveGoalPosition();
            //obstacle on right side
            if (message->data == 1)
            {
                //select new heading 0.2 radians to the left
                goalLocation.theta = currentLocation.theta + 0.78; //0.78
            }

            //obstacle in front or on left side
            else if (message->data == 2)
            {
                //select new heading 0.2 radians to the right
                goalLocation.theta = currentLocation.theta - 0.78; //0.78
            }
            else
            {
                //select new heading 0.2 radians to the left
                goalLocation.theta = currentLocation.theta + 0.78; //0.78
            }

            //select new position 0.50 m from current location
            goalLocation.x = currentLocation.x + (0.75 * cos(goalLocation.theta));
            goalLocation.y = currentLocation.y + (0.75 * sin(goalLocation.theta));
        }
        avoiding_obstacle = true;

        //switch to reset rotate pid to trigger collision avoidance
        stateMachineState = STATE_MACHINE_ROTATE;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulationMode == 0 || simulationMode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!publishedName)
    {
        packageMessage("I "+ roverName);
        publishedName = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr &message)
{
    targetsCollected[message->data] = 1;
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

/***********************
 * UTILITY FUNCTIONS
************************/
double computeGoalTheta(geometry_msgs::Pose2D goalLocation, geometry_msgs::Pose2D currentLocation)
{
    return atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
}

double computeDistanceToGoal(geometry_msgs::Pose2D goalLocation, geometry_msgs::Pose2D currentLocation)
{
    return fabs(sqrt( (pow( (goalLocation.x - currentLocation.x), 2.0)) + (pow( (goalLocation.y - currentLocation.y) , 2.0)) ));
}

void setGoalLocation(double goalLocationX, double goalLocationY)
{
    goalLocation.x = goalLocationX;
    goalLocation.y = goalLocationY;
    goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
}

void reportDetected(std_msgs::Int16 msg) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    int targets_detected_size = 0;
    for(int i = 0; i < 256; i++)
    {
        if(targetsDetected[i])
        {
            targets_detected_size++;
        }
    }
    std::stringstream converter;
    
    converter <<
                 roverName << " " <<
                 msg.data << " " <<
                 currentLocation.x << " " <<
                 currentLocation.y << " " <<
                 currentLocation.theta << " " <<
                 current_time-timeStampTransitionToAuto << " " <<
                 targets_detected_size;
    packageMessage("D "+converter.str());
}

void claimResource(int resouceID) // simply publish (broadcast) 
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << roverName << " " << current_time-timeStampTransitionToAuto;
    packageMessage("C "+converter.str());
}

void unclaimResource(int resouceID) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << roverName << " " << current_time-timeStampTransitionToAuto;
    packageMessage("U "+converter.str());
}

void homeResource(int resouceID) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    int targets_home_size = 0;

    for(int i = 0; i < 256; i++)
    {
        if(targetsHome[i])
        {
            targets_home_size++;
        }
    }

    std::stringstream converter;
    converter << resouceID << " " << roverName << " " << current_time-timeStampTransitionToAuto << " " << targets_home_size;
    packageMessage("H "+converter.str());
}

void packageMessage(string message_content)
{
    std_msgs::String message;
    message.data = message_content;
    message_publisher.publish(message);
}

// search detected targets and find the shortest distance to your current position
int searchQueue()
{
    double minDistance = LONG_MAX;
    int idClosestTarget = -1;
    for(int resource = 0; resource < 256; resource++)
    {
        if(targetsAvailDetected[resource])
        {

            double current_distance = computeDistanceToGoal(targetPositions[resource], currentLocation);
            if(current_distance < minDistance)
            {
                minDistance = current_distance;
                idClosestTarget = resource;
            }
        }
    }
    return idClosestTarget;
}

/***********************
 * CUSTOM MESSAGE HANDLERS
 ************************/
void messageHandler(const std_msgs::String::ConstPtr& message)
{
    string msg = message->data;

    size_t type_pos = msg.find_first_of(" ");
    string type = msg.substr(0, type_pos);
    msg = msg.substr(type_pos+1);

    vector<string> msg_parts;

    size_t cur_tok = msg.find_first_of(" ");;
    while(cur_tok != string::npos) { // until end of string
        msg_parts.push_back(msg.substr(0, cur_tok));
        msg = msg.substr(cur_tok + 1);
        cur_tok = msg.find_first_of(" ");
    }

    msg_parts.push_back(msg);

    if(type == "D")
    {
        detectedMessage(msg_parts);
    }
    else if(type == "C")
    {
        claimMessage(msg_parts);
    }
    else if(type == "U")
    {
        unclaimMessage(msg_parts);
    }
    else if(type == "H")
    {
        homeMessage(msg_parts);
    }
}

void detectedMessage(vector<string> msg_parts)
{
    std_msgs::Int16 tmp;
    tmp.data = -1;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    string name = "";

    stringstream converter;

    converter << msg_parts[0];
    converter >> name;
    converter.str("");
    converter.clear();

    converter << msg_parts[1];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    converter << msg_parts[2];
    converter >> x;
    converter.str("");
    converter.clear();

    converter << msg_parts[3];
    converter >> y;
    converter.str("");
    converter.clear();

    converter << msg_parts[4];
    converter >> theta;
    converter.str("");
    converter.clear();

    // lookup tables
    targetPositions[tmp.data].x = x + 0.75 * cos(theta); // project forward by 0.5 meters in the direction that the finding rover was facing when the target was detected
    targetPositions[tmp.data].y = y + 0.75 * sin(theta);
    targetPositions[tmp.data].theta = theta;
    targetsDetected[tmp.data] = true;
    targetsAvailDetected[tmp.data] = targetsDetected[tmp.data] && targetsAvailable[tmp.data];
    if ( (roverCurrentMode == MODE_COLLECTOR) && (roverCapacity == CAPACITY_EMPTY) )
    {
        stateMachineState = STATE_MACHINE_CHANGE_MODE;
    }
}

void claimMessage(vector<string> msg_parts) // claimed this, now remove from global queue
{
    std_msgs::Int16 tmp;
    tmp.data = -1;

    stringstream converter;

    converter << msg_parts[0];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    targetsAvailable[tmp.data] = false;
    targetsAvailDetected[tmp.data] = targetsDetected[tmp.data] && targetsAvailable[tmp.data];
}

void unclaimMessage(vector<string> msg_parts) // unclaimed this, now add back to global queue
{
    std_msgs::Int16 tmp;
    tmp.data = -1;

    stringstream converter;

    converter << msg_parts[0];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    targetsAvailable[tmp.data] = true;
    targetsAvailDetected[tmp.data] = targetsDetected[tmp.data] && targetsAvailable[tmp.data];
}

void homeMessage(vector<string> msg_parts) // unclaimed this, now add back to global queue
{
    std_msgs::Int16 tmp;
    tmp.data = -1;

    stringstream converter;

    converter << msg_parts[0];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    targetsHome[tmp.data] = true;
}

void visitRandomLocation()
{
    double r = rng->uniformReal(0.5, 6);
    double t = rng->uniformReal(0, 2 * M_PI);
    goalLocation.x = r * cos(t);
    goalLocation.y = r * sin(t);
    goalLocation.theta = computeGoalTheta(goalLocation, currentLocation);
}

