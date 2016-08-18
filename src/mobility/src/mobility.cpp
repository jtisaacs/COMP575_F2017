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

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>
#include <math.h> 

using namespace std;

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

// Random number generator
random_numbers::RandomNumberGenerator *rng;

// Numeric Variables
string publishedName;
char host[128];
bool sent_name = false;

char prev_state_machine[128];
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
geometry_msgs::Twist velocity;

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
vector <geometry_msgs::Pose2D> achilles_waypoints;

// ajax and aeneas waypoints, lawnmower search
const int LAWNMOWER_SIZE_1 = 22; //29JTI must change this to match size of waypoints_x.  Why??
const int LAWNMOWER_SIZE_2 = 35;
//float waypoints_x [] = {6.5, 6.5, -6.5, -6.5, 6, 6, -6.5, -6.5, 6, 6, -6.5, -6.5, 6, 6, -6.5, -6.5, 6, 6, -6.5, -6.5, 6, 6, -6.5, -6.5, 6, 6, -6.5, -6.5,0,0};
//float waypoints_y [] = {0, 6.5, 6.5, 6, 6, 5.5, 5.5, 5, 5, 4.5, 4.5, 4, 4, 3.5, 3.5, 3, 3, 2.5, 2.5, 2, 2, 1.5, 1.5, 1, 1, .5, .5, 0,0,0};
//float waypoints_x [] = {7.0, 7.0, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, -7.0, -7.0, 6.5, 6.5, 0};
//float waypoints_y [] = {  0, 7.0,  7.0,  6.5, 6.5, 6.0,  6.0,  5.5, 5.5, 5.0,  5.0,  4.5, 4.5,   4,    4,  3.5, 3.5,   3,    3,  2.5, 2.5,   2,    2,  1.5, 1.5,   1,    1,  0.5, 0.5,   0, 0};

const float d = 0.5;
float waypoints_x [] = {7.5-d, -(7.5-d), -(7.5-d),    7.5-d,   7.5-d, -(7.5-2*d), -(7.5-2*d),    7.5-2*d, 7.5-2*d, -(7.5-3*d), -(7.5-3*d),    7.5-3*d, 7.5-3*d, -(7.5-4*d), -(7.5-4*d),   7.5-4*d, 7.5-4*d, 7.5-4*d, -(7.5-5*d), -(7.5-5*d),    7.5-5*d,    7.5-5*d};
float waypoints_y [] = {7.5-d,    7.5-d, -(7.5-d), -(7.5-d), 7.5-2*d,    7.5-2*d, -(7.5-2*d), -(7.5-2*d), 7.5-3*d,    7.5-3*d, -(7.5-3*d), -(7.5-3*d), 7.5-4*d,    7.5-4*d, -(7.5-4*d), -(7.5-4*d), 7.5-5*d, 7.5-5*d,    7.5-5*d, -(7.5-5*d), -(7.5-5*d),    7.5-6*d};

float waypoints_x2 [] = {7.5-5*d, -(7.5-6*d), -(7.5-6*d),    7.5-6*d, 7.5-6*d, 7.5-6*d, -(7.5-7*d), -(7.5-7*d),    7.5-7*d, 7.5-7*d, 7.5-7*d, -(7.5-8*d), -(7.5-8*d),    7.5-8*d, 7.5-8*d, 7.5-8*d, -(7.5-9*d), -(7.5-9*d),   7.5-9*d,   7.5-9*d, 7.5-9*d, -(7.5-10*d), -(7.5-10*d),   7.5-10*d,   7.5-10*d, 7.5-10*d, -(7.5-11*d), -(7.5-11*d),    7.5-11*d, 7.5-11*d, 7.5-11*d, -(7.5-12*d), -(7.5-12*d),    7.5-12*d, 7.5-12*d};
float waypoints_y2 [] = {7.5-6*d,    7.5-6*d, -(7.5-6*d), -(7.5-6*d), 7.5-7*d, 7.5-7*d,    7.5-7*d, -(7.5-7*d), -(7.5-7*d), 7.5-8*d, 7.5-8*d,    7.5-8*d, -(7.5-8*d), -(7.5-8*d), 7.5-9*d, 7.5-9*d,    7.5-9*d, -(7.5-9*d), -(7.5-9*d), 7.5-10*d, 7.5-10*d,    7.5-10*d, -(7.5-10*d), -(7.5-10*d), 7.5-11*d, 7.5-11*d,    7.5-11*d, -(7.5-11*d), -(7.5-11*d), 7.5-12*d, 7.5-12*d,    7.5-12*d, -(7.5-12*d), -(7.5-12*d), 7.5-13*d};

vector <geometry_msgs::Pose2D> ajax_waypoints;
vector <geometry_msgs::Pose2D> aeneas_waypoints;

// ajax and aeneas cluster detection
int targets_detected_screenshot = 0;

int transitionsToAuto = 0;
double timeStampTransitionToAuto = 0.0;

// sorted queue
vector <geometry_msgs::Pose2D> queue;

// theta PID controller - rotate
double r_theta_error = 0;
double r_theta_error_prior = 0;
double r_theta_integral = 0;

// theta PID controller - translate
double t_theta_error = 0;
double t_theta_error_prior = 0;
double t_theta_integral = 0;

// velocity PID controller
double velocity_error = 0;
double velocity_error_prior = 0;
double velocity_integral = 0;

#define LOGIC_INIT          0
#define LOGIC_SEARCH        1
#define LOGIC_COLLECT_RWALK 2
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
#define STATE_MACHINE_RESET_ROTATE_PID 10
#define STATE_MACHINE_RESET_TRANSLATE_PID 11
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
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;

ros::Publisher messagePublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;
ros::Subscriber messageSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

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
double velocityPID(double KP, double KI, double KD);
double r_thetaPID(double KP, double KI, double KD);
double t_thetaPID(double KP, double KI, double KD);
double distance_(double x1, double x2, double y1, double y2);

int searchQueue();

void debugWaypoints();
void debugRotate();
void debugTranslate(double distance_);
void debugRandom();
void visitRandomLocation();
void claimResource(int resouceID);
void unclaimResource(int resouceID);
void homeResource(int resouceID);

bool cluster = false;
void circle();
void lookBack();

void debugTargetScreenshot(int count);

void reportDetected(std_msgs::Int16 msg);

void detectedMessage(vector<string> msg_parts);
void claimMessage(vector<string> msg_parts);
void unclaimMessage(vector<string> msg_parts);
void homeMessage(vector<string> msg_parts);

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostname(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    targetClaimed.data = -1; // initialize target claimed
    claimedTargetDetected.data = -1; // intially the gatherer has no claimed targets so -1 indicates that the gatherer is either looking for a claimed target or there is no current claimed target.

    if (argc >= 2)
    {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else
    {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
    targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);

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

    if ((currentMode == 2 || currentMode == 3)) // Robot is in automode
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
            if (publishedName == "ajax" || publishedName == "aeneas") // set mode to searcher
            {
                roverCurrentMode = MODE_SEARCHER; // set ajax and aeneas to be the lawnmower path searchers.

                if (publishedName == "ajax") // assign waypoints to perform lawnmower
                {
                    // init local waypoints
                    goalLocation.x =  waypoints_x[0];
                    goalLocation.y = waypoints_y[0];
                    goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                    geometry_msgs::Pose2D nextPosition;
                    for (int i = LAWNMOWER_SIZE_1; i > 0; i--)
                    {
                        nextPosition.x = waypoints_x[i];
                        nextPosition.y = waypoints_y[i];
                        ajax_waypoints.push_back(nextPosition);
                    }
                }
                else if (publishedName == "aeneas") // assign waypoints to perform lawnmower
                {
                    // init local waypoints
                    goalLocation.x = waypoints_x2[0];
                    goalLocation.y = waypoints_y2[0];
                    goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                    geometry_msgs::Pose2D nextPosition;
                    for (int i = LAWNMOWER_SIZE_2; i > 0; i--)
                    {
//                        nextPosition.x = (-1)*waypoints_x[i];
//                        nextPosition.y = (-1)*waypoints_y[i];
                        nextPosition.x = waypoints_x2[i];
                        nextPosition.y = waypoints_y2[i];
                        aeneas_waypoints.push_back(nextPosition);
                    }
                }
            }
            else // all other robots, assign random point to search and collect
            {
                roverCurrentMode = MODE_COLLECTOR;
//                visitRandomLocation();
                goalLocation.x = 0.0;
                goalLocation.y = 0.0;
                goalLocation.theta = M_PI;
            }
            stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
            break;
        }
        case STATE_MACHINE_RESET_ROTATE_PID:
        {
            stateMachineMsg.data = "RESETTING ROTATE PID";
            r_theta_integral = 0;
            stateMachineState = STATE_MACHINE_ROTATE;
            break;
        }
        case STATE_MACHINE_ROTATE:
        {
            double angularVelocity = 0.0;
            double randVel = rng->uniformReal(-.02, 0.02);
//            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
//            if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.2)
            if (angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x)) > 0.25)
            {
                // double angularVelocity = r_thetaPID(1.55, 0.2, 0);
                angularVelocity = r_thetaPID(0.75, 0.0, 0.0); //JTI Tuned KI to match the negative rotation case.
                if(angularVelocity > MAX_ANGULAR_VELOCITY)
                {
                    angularVelocity = MAX_ANGULAR_VELOCITY;
                }
                setVelocity(randVel, angularVelocity); //add a little bit of random linear velocity to make turns less sticky
            }
//            else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.2)
            else if (angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x)) < -0.25)
            {
                angularVelocity = r_thetaPID(0.75, 0.0, 0.0);
                if(angularVelocity < -MAX_ANGULAR_VELOCITY)
                {
                    angularVelocity = -MAX_ANGULAR_VELOCITY;
                }
                setVelocity(randVel, angularVelocity);//add a little bit of random linear velocity to make turns less sticky
            }
            else
            {
                angularVelocity = 0.0;
                setVelocity(0.0, angularVelocity); // stop
                stateMachineState = STATE_MACHINE_RESET_TRANSLATE_PID; // move to translate step
            }
            std::stringstream converter;
            converter <<
                         angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) << ", " <<
                         angularVelocity;

            stateMachineMsg.data = "ROTATING";//, " + converter.str();
            break;
        }
        case STATE_MACHINE_RESET_TRANSLATE_PID:
        {
            stateMachineMsg.data = "RESETTING TRANSLATE PID";
            t_theta_integral = 0;
            velocity_integral = 0;
            stateMachineState = STATE_MACHINE_TRANSLATE;
            break;
        }
        case STATE_MACHINE_TRANSLATE:
        {

            double angularVelocity = 0.0;
            double linearVelocity = 0.0;


            double calc_distance = distance_(goalLocation.x, currentLocation.x, goalLocation.y, currentLocation.y);
//            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
//            if ((fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)||(calc_distance >= 0.25)) // if you are not within 25cm of goal
            if (calc_distance >= 0.25) // if you are not within 25cm of goal
            {
                angularVelocity = t_thetaPID(0.50, 0.0, 0.0); // needed to keep robot going straight
                linearVelocity = velocityPID(0.2, 0.0, 0.0);//0.15

                if(angularVelocity > MAX_ANGULAR_VELOCITY)
                {
                    angularVelocity = MAX_ANGULAR_VELOCITY;
                }
                else if(angularVelocity < -MAX_ANGULAR_VELOCITY)
                {
                    angularVelocity = -MAX_ANGULAR_VELOCITY;
                }
                double max_Remaining_Velocity = MAX_LINEAR_VELOCITY;//-fabs(angularVelocity); // We can only supply 0.3 velocity in total divided between angular and linear.  Give priorty to angular.
                if(linearVelocity > max_Remaining_Velocity)
                {
                    linearVelocity = max_Remaining_Velocity;
                }
                else if (linearVelocity < -max_Remaining_Velocity)
                {
                    linearVelocity = -max_Remaining_Velocity;
                }
                setVelocity(linearVelocity, angularVelocity);
                //stateMachineState = STATE_MACHINE_TRANSLATE; // added just for readability.  We repeat this state until we reach goal location.
            }
            else
            {
                angularVelocity = 0.0;
                linearVelocity = 0.0;
                setVelocity(linearVelocity, angularVelocity);

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
                    goalLocation.theta = savedPositions.back().theta;
                    //                    pivot();
                    savedPositions.pop_back();
                    stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (publishedName == "ajax") && (ajax_waypoints.size() > 0) )
                {
                    stateMachineState = STATE_MACHINE_POP_WAYPOINT; // We are ajax and we are currently searching with remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (publishedName == "aeneas") && (aeneas_waypoints.size() > 0) )
                {
                    stateMachineState = STATE_MACHINE_POP_WAYPOINT; // We are aeneas and we are currently searching with remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (publishedName == "ajax") && (ajax_waypoints.size() <= 0) )
                {
                    stateMachineState = STATE_MACHINE_CHANGE_MODE; // We are ajax and we are currently searching with no remaining waypoints
                }
                else if ( (roverCurrentMode == MODE_SEARCHER) && (publishedName == "aeneas") && (aeneas_waypoints.size() <= 0) )
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
//            converter <<
//                         "Distance: " << calc_distance << " AngularVel: " << angularVelocity << " LinearVel: " << linearVelocity << " GoalLocationX: " << goalLocation.x << " CurrentLocationX: " << currentLocation.x << " GoalLocationY: " << goalLocation.y << " CurrentLocationY: " << currentLocation.y;
            converter <<
                         "G_T: " << goalLocation.theta << " C_T: " << currentLocation.theta << " A_V: " << angularVelocity << " Dist: " << calc_distance << " L_V: " << linearVelocity << " G_X: " << goalLocation.x << " G_Y: " << goalLocation.y;

            stateMachineMsg.data = "TRANSLATING";//, " + converter.str();
            break;
        }
        case STATE_MACHINE_POP_WAYPOINT:
        {
            stateMachineMsg.data = "POPPING WAYPOINT";
            if (publishedName == "ajax")
            {
                goalLocation.x = ajax_waypoints.back().x;
                goalLocation.y = ajax_waypoints.back().y;
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                ajax_waypoints.pop_back();
            }
            if (publishedName == "aeneas")
            {
                goalLocation.x = aeneas_waypoints.back().x;
                goalLocation.y = aeneas_waypoints.back().y;
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                aeneas_waypoints.pop_back();
            }
            stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                //            goalLocation.x = currentLocation.x;
                //            goalLocation.y = currentLocation.y;
                //            goalLocation.theta = atan2(currentLocation.y, currentLocation.x) + rng->uniformReal(0, 2 * M_PI); // Since original rover saw the april tag from this (x,y) position we should rotate to a random heading till we see it.
                stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
            stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
            stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
            break;
        }
        case STATE_MACHINE_EXPLORE_NEAR_ORIGIN:
        {
            stateMachineMsg.data = "EXPORING NEAR ORIGIN";
            double r = rng->uniformReal(0.0, 1);
            double t = rng->uniformReal(0, 2 * M_PI);
            goalLocation.x = 0.0 + r * cos(t); // Explore in a 1m circle around where you think the target is located.
            goalLocation.y = 0.0 + r * sin(t);
            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
//            goalLocation.x = currentLocation.x;
//            goalLocation.y = currentLocation.y;
//            goalLocation.theta = atan2(currentLocation.y, currentLocation.x) + rng->uniformReal(0, 2 * M_PI); // Since original rover saw the april tag from this (x,y) position we should rotate to a random heading till we see it.
            stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
        converter <<
                     "CURRENT MODE: " << currentMode;

        stateMachineMsg.data = "WAITING, " + converter.str();
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
    {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }

}

void setVelocity(double linearVel, double angularVel)
{
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
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
                targetDropOffPublish.publish(message->image); // publish the image that you are dropping off so it can be scored.

                //targetsCollected[targetClaimed.data] = 1;

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
                targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                // publish detected target
                targetCollectedPublish.publish(claimedTargetDetected);
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

                    unclaimResource(targetClaimed.data);

                    claimResource(message->tags.data[i]); // update targets detected and available array
                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.


                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    targetCollectedPublish.publish(claimedTargetDetected);
                    // targetCollectedPublish.publish(targetDetected); // from UNM Code
                    stateMachineState = STATE_MACHINE_RETURN_HOME;
                }
                else if ( !targetsDetected[message->tags.data[i]] )
                {
                    // here we need to unclaim our original, then reportDectected and claim this one.
                    setVelocity(0.0,0.0); // stop the rover
                    roverCapacity=CAPACITY_CARRYING; // set the capacity of this rover to carrying

                    unclaimResource(targetClaimed.data);

                    std_msgs::Int16 tag;
                    tag.data = message->tags.data[i];
                    reportDetected(tag);

                    claimResource(message->tags.data[i]); // update targets detected and available array
                    targetClaimed.data = message->tags.data[i]; // This should be where targetClaimed gets set.


                    claimedTargetDetected.data = message->tags.data[i];
                    // publish to scoring code
                    targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    targetCollectedPublish.publish(claimedTargetDetected);
                    // targetCollectedPublish.publish(targetDetected); // from UNM Code
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
                    targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    targetCollectedPublish.publish(claimedTargetDetected);
                    // targetCollectedPublish.publish(targetDetected); // from UNM Code
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
                    targetPickUpPublish.publish(message->image); //publish the image that you are picking up.
                    // publish detected target
                    targetCollectedPublish.publish(claimedTargetDetected);
                    // targetCollectedPublish.publish(targetDetected); // from UNM Code
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
    currentMode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( (message->data > 0) && roverCurrentMode==MODE_COLLECTOR ) //!(avoiding_obstacle)
    {
//        setVelocity(0.0,0.0); // stop the rover
        if ( (roverCapacity==CAPACITY_CARRYING) && (angles::shortest_angular_distance(currentLocation.theta, atan2(-currentLocation.y, -currentLocation.x)) < M_PI_2) )
        {
            setVelocity(-0.2,0.0);
            // we are going to the goal with a target.  Just wait till the other guy moves out of our way.
            geometry_msgs::Pose2D savedPosition;

            savedPosition.x = goalLocation.x;
            savedPosition.y = goalLocation.y;
            savedPosition.theta = goalLocation.theta;

            savedPositions.push_back(savedPosition);

            //try not to move.
            goalLocation.x = currentLocation.x;
            goalLocation.y = currentLocation.y;
            goalLocation.theta = currentLocation.theta;


        }
        else
        {
            setVelocity(-0.2,0.0);
            // we are not delivering a target so lets get out of the way.
            geometry_msgs::Pose2D savedPosition;

            savedPosition.x = goalLocation.x;
            savedPosition.y = goalLocation.y;
            savedPosition.theta = goalLocation.theta;

            savedPositions.push_back(savedPosition);

            //obstacle on right side
            if (message->data == 1)
            {
                //select new heading 0.2 radians to the left
                goalLocation.theta = currentLocation.theta + 0.78; //0.78
//                setVelocity(0.2,-0.3);
            }

            //obstacle in front or on left side
            else if (message->data == 2)
            {
                //select new heading 0.2 radians to the right
                goalLocation.theta = currentLocation.theta - 0.78; //0.78
//                setVelocity(0.2,0.3);
            }
            else
            {
                //select new heading 0.2 radians to the left
                goalLocation.theta = currentLocation.theta + 0.78; //0.78
//                setVelocity(0.2,-0.3);
            }

            //select new position 0.50 m from current location
            goalLocation.x = currentLocation.x + (0.75 * cos(goalLocation.theta));
            goalLocation.y = currentLocation.y + (0.75 * sin(goalLocation.theta));
        }
        avoiding_obstacle = true;

        //switch to reset rotate pid to trigger collision avoidance
        stateMachineState = STATE_MACHINE_RESET_ROTATE_PID;
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
    if (currentMode == 0 || currentMode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!sent_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + publishedName;
        messagePublish.publish(name_msg);
        sent_name = true;
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
double distance_(double x1, double x2, double y1, double y2)
{
    return fabs(sqrt( (pow( (x2 - x1), 2.0)) + (pow( (y2 - y1) , 2.0)) ));
}

// theta PID controller for rotate state
double r_thetaPID(double KP, double KI, double KD)
{
//    double theta_desired = goalLocation.theta; //atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
//    r_theta_error = theta_desired - currentLocation.theta;
//    r_theta_error = theta_desired - atan2(currentLocation.y,currentLocation.x);
//    r_theta_error = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
    r_theta_error = angles::shortest_angular_distance(currentLocation.theta,atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x));
    double theta_error_prime = atan2(sin(r_theta_error), cos(r_theta_error));
    r_theta_integral = r_theta_integral + theta_error_prime;
    double theta_derivative = (theta_error_prime - r_theta_error_prior) / mobilityLoopTimeStep;
    double theta_output = (KP*theta_error_prime) + (KI*r_theta_integral) + (KD*theta_derivative);
//    r_theta_error_prior = r_theta_error;
    r_theta_error_prior = theta_error_prime;
    return theta_output;
}

// theta PID controller for translate state
double t_thetaPID(double KP, double KI, double KD)
{
//    t_theta_error = goalLocation.theta - currentLocation.theta;
    t_theta_error = angles::shortest_angular_distance(currentLocation.theta,atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x));
    double theta_error_prime = atan2(sin(t_theta_error), cos(t_theta_error));
    t_theta_integral = t_theta_integral + theta_error_prime;
    double theta_derivative = (theta_error_prime - t_theta_error_prior) / mobilityLoopTimeStep;
    double theta_output = (KP*theta_error_prime) + (KI*t_theta_integral) + (KD*theta_derivative);
//    t_theta_error_prior = t_theta_error;
    t_theta_error_prior = theta_error_prime;
    return theta_output;
}

// velocity PID controller
double velocityPID(double KP, double KI, double KD)
{
    velocity_error = distance_(goalLocation.x, currentLocation.x, goalLocation.y, currentLocation.y);
    velocity_integral = velocity_integral + velocity_error;
    double velocity_derivative = (velocity_error - velocity_error_prior) / mobilityLoopTimeStep;
    double velocity_output = (KP*velocity_error) + (KI*velocity_integral) + (KD*velocity_derivative);
    velocity_error_prior = velocity_error;
    return velocity_output;
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
                 publishedName << " " <<
                 msg.data << " " <<
                 currentLocation.x << " " <<
                 currentLocation.y << " " <<
                 currentLocation.theta << " " <<
                 current_time-timeStampTransitionToAuto << " " <<
                 targets_detected_size;
    
    std_msgs::String message;
    message.data = "D " + converter.str();
    messagePublish.publish(message);
}

void claimResource(int resouceID) // simply publish (broadcast) 
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << publishedName << " " << current_time-timeStampTransitionToAuto;
    std_msgs::String message;
    message.data = "C " + converter.str(); // claim this token for pickup
    messagePublish.publish(message);
}

void unclaimResource(int resouceID) // simply publish (broadcast)
{
    double current_time = ros::Time::now().toSec();
    std::stringstream converter;
    converter << resouceID << " " << publishedName << " " << current_time-timeStampTransitionToAuto;
    std_msgs::String message;
    message.data = "U " + converter.str(); // unclaim this token for pickup
    messagePublish.publish(message);
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
    converter << resouceID << " " << publishedName << " " << current_time-timeStampTransitionToAuto << " " << targets_home_size;
    std_msgs::String message;
    message.data = "H " + converter.str(); // unclaim this token for pickup
    messagePublish.publish(message);
}

// search detected targets and find the shortest distance to your current position
int searchQueue()
{
    double found_x = 0;
    double found_y = 0;
    double found_theta = 0;
    double minDistance = LONG_MAX;
    int idClosestTarget = -1;

    for(int resource = 0; resource < 256; resource++)
    {
        if(targetsAvailDetected[resource])
        {
            found_x = targetPositions[resource].x;
            found_y = targetPositions[resource].y;
            double current_distance = distance_(found_x, currentLocation.x, found_y, currentLocation.y);
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
//    int targets_detected_size = 0;
//    int targets_available_size = 0;
//    int targets_available_detected_size = 0;
//    int targets_collected_size = 0;
//    for(int i = 0; i < 256; i++)
//    {
//        if(targetsDetected[i])
//        {
//            targets_detected_size++;
//        }
//        if(targetsAvailable[i])
//        {
//            targets_available_size++;
//        }
//        if(targetsAvailDetected[i])
//        {
//            targets_available_detected_size++;
//        }
//        if(targetsCollected[i])
//        {
//            targets_collected_size++;
//        }
//    }

//    std::stringstream temp_data;
//    temp_data <<
//                 publishedName << " targetsAvailDetected " << targets_available_detected_size << " targetsDetected " << targets_detected_size << " targetsAvailable " << targets_available_size << " targetsCollected " << targets_collected_size   ;
//    std_msgs::String message;
//    message.data = "DETECTED " + temp_data.str();
//    messagePublish.publish(message);
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
//    targetClaimed.data = tmp.data; // sets the id of the claimed target to be used by target handler.

//    int targets_detected_size = 0;
//    int targets_available_size = 0;
//    int targets_available_detected_size = 0;
//    int targets_collected_size = 0;

//    for(int i = 0; i < 256; i++)
//    {
//        if(targetsDetected[i])
//        {
//            targets_detected_size++;
//        }
//        if(targetsAvailable[i])
//        {
//            targets_available_size++;
//        }
//        if(targetsAvailDetected[i])
//        {
//            targets_available_detected_size++;
//        }
//        if(targetsCollected[i])
//        {
//            targets_collected_size++;
//        }
//    }

//    std::stringstream temp_data;
//    temp_data <<
//                 publishedName << " targetsAvailDetected " << targets_available_detected_size << " targetsDetected " << targets_detected_size << " targetsAvailable " << targets_available_size << " targetsCollected " << targets_collected_size;
//    std_msgs::String message;
//    message.data = "CLAIM " + temp_data.str();
//    messagePublish.publish(message);
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
//    targetClaimed.data = tmp.data; // sets the id of the claimed target to be used by target handler.

//    int targets_detected_size = 0;
//    int targets_available_size = 0;
//    int targets_available_detected_size = 0;
//    int targets_collected_size = 0;

//    for(int i = 0; i < 256; i++)
//    {
//        if(targetsDetected[i])
//        {
//            targets_detected_size++;
//        }
//        if(targetsAvailable[i])
//        {
//            targets_available_size++;
//        }
//        if(targetsAvailDetected[i])
//        {
//            targets_available_detected_size++;
//        }
//        if(targetsCollected[i])
//        {
//            targets_collected_size++;
//        }
//    }

//    std::stringstream temp_data;
//    temp_data <<
//                 publishedName << " targetsAvailDetected " << targets_available_detected_size << " targetsDetected " << targets_detected_size << " targetsAvailable " << targets_available_size << " targetsCollected " << targets_collected_size;
//    std_msgs::String message;
//    message.data = "UNCLAIM " + temp_data.str();
//    messagePublish.publish(message);
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
//    targetClaimed.data = tmp.data; // sets the id of the claimed target to be used by target handler.

//    int targets_detected_size = 0;
//    int targets_available_size = 0;
//    int targets_available_detected_size = 0;
//    int targets_home_size = 0;

//    for(int i = 0; i < 256; i++)
//    {
//        if(targetsDetected[i])
//        {
//            targets_detected_size++;
//        }
//        if(targetsAvailable[i])
//        {
//            targets_available_size++;
//        }
//        if(targetsAvailDetected[i])
//        {
//            targets_available_detected_size++;
//        }
//        if(targetsHome[i])
//        {
//            targets_home_size++;
//        }
//    }

//    std::stringstream temp_data;
//    temp_data <<
//                 publishedName << " targetsAvailDetected " << targets_available_detected_size << " targetsDetected " << targets_detected_size << " targetsAvailable " << targets_available_size << " targetsHome " << targets_home_size;
//    std_msgs::String message;
//    message.data = "HOME " + temp_data.str();
//    messagePublish.publish(message);
}
/***********************
 * DEBUGGING
 ************************/
void debugWaypoints()
{
    std::stringstream converter;
    converter <<
                 publishedName << " cx" << currentLocation.x << " cy" << currentLocation.y << " "  << " gx" << goalLocation.x << " gy" << goalLocation.y;
    std_msgs::String message;
    message.data = "WAYPOINTS " + converter.str();
    messagePublish.publish(message);
}

void debugRotate()
{
    std::stringstream converter;
    converter <<
                 publishedName << " ct" << currentLocation.theta << " gt" << goalLocation.theta;
    std_msgs::String message;
    message.data = "ROTATING " + converter.str();
    messagePublish.publish(message);
}

void debugTranslate(double distance_)
{
    std::stringstream converter;
    converter <<
                 publishedName << " cx" << currentLocation.x << " cy" << currentLocation.y << " "  << " gx" << goalLocation.x << " gy" << goalLocation.y << " " << distance_;
    std_msgs::String message;
    message.data = "TRANSLATING " + converter.str();
    messagePublish.publish(message);
}

void debugRandom()
{
    int targets_detected_size = 0;
    int count = 0;
    for(int i = 0; i < 256; i++)
    {
        if(targetsDetected[i])
        {
            count++;
        }
        if(targetsAvailDetected[i])
        {
            targets_detected_size++;
        }
    }


    std::stringstream converter;
    converter <<
                 publishedName << " targetsAvailDetected_size " << targets_detected_size << "TOTAL: " << count;
    std_msgs::String message;
    message.data = "RANDOM " + converter.str();
    messagePublish.publish(message);
}

void debugTargetScreenshot(int count)
{
    std::stringstream converter;
    converter <<
                 publishedName << " " << count;
    std_msgs::String message;
    message.data = "COUNT " + converter.str();
    messagePublish.publish(message);
}

void visitRandomLocation()
{
    double r = rng->uniformReal(0.5, 6);
    double t = rng->uniformReal(0, 2 * M_PI);
    goalLocation.x = r * cos(t);
    goalLocation.y = r * sin(t);
    goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
}

void lookBack()
{
    geometry_msgs::Pose2D savedPosition;
    savedPosition.x = goalLocation.x;
    savedPosition.y = goalLocation.y;
    savedPosition.theta = goalLocation.theta;
    cluster_detect.push_back(savedPosition);

    // goalLocation.theta = currentLocation.theta - 0.2; // move to the right
    // goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
    // goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
    // setVelocity(0.0, 0.0);

    // new goal locations
    goalLocation.x = .25*cos(currentLocation.theta + M_PI) + currentLocation.x;
    goalLocation.y = .25*sin(currentLocation.theta + M_PI) + currentLocation.y;
    goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
    

//    pivot();


    // goalLocation.x = currentLocation.x;
    // goalLocation.y = currentLocation.y;
    // goalLocation.theta = currentLocation.theta - M_PI;

    std::stringstream converter;
    converter <<
                 publishedName << " cx " << currentLocation.x << " cy " << currentLocation.y << " ct " << currentLocation.theta <<
                 " gx " << goalLocation.x << " gy " << goalLocation.y << " gt " << goalLocation.theta;
    std_msgs::String message;
    message.data = "CLUSTER " + converter.str();
    messagePublish.publish(message);

    cluster = true;
    // stateMachineState = STATE_MACHINE_TRANSFORM;
}

void circle()
{
    double temp_x = goalLocation.x;
    double temp_y = goalLocation.y;
    double temp_theta = goalLocation.theta;
    for(int i = 0; i < 3; i++)
    {
        goalLocation.x = 0.001*cos(currentLocation.theta + M_PI);
        goalLocation.y = 0.001*sin(currentLocation.theta + M_PI);
        goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
        // goalLocation.theta = angles::shortest_angular_distance(currentLocation.theta, M_PI); // flip directions
        // goalLocation.theta = currentLocation.theta - M_PI; // flip directions
//        pivot(); // rotate backwards
        goalLocation.x = temp_x;
        goalLocation.y = temp_y;
        goalLocation.theta = temp_theta;
//        pivot(); // rotate to original heading
    }
    targets_detected_screenshot = 0;
}
