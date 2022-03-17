#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include "tf/transform_listener.h" 
#include <nav_msgs/Odometry.h>
#include  <stdlib.h>
#include <iostream>
#include <chrono>
using namespace std::chrono;

ros::Publisher pub;
ros::Subscriber sub;

geometry_msgs::Twist cmd;
sensor_msgs::LaserScan laser_msg;
float min_right, min_front;
float d = 0.6; // optimal distance to the wall
bool first=true;
bool s = false;
high_resolution_clock::time_point start;
void moveDecision();


//Behaviour of the robot
typedef enum _ROBOT_MOVEMENT {
    FIND_THE_WALL,
    TURN_LEFT,
    FOLLOW_WALL,
    STOP,
    BACKWARD,
    FORWARD,

} ROBOT_MOVEMENT;

ROBOT_MOVEMENT currentState = FORWARD;

// Find minimum distance in the array of ranges
float getMin(float arr[], int n)
{
   float min = arr[0];
   for(int i=0; i<n; i++) {
      if(min>arr[i]) {
         min=arr[i];
      }
   }
   return min;
}


void StateMachine()
{
    switch(currentState)
    {
        case FORWARD: 
            ROS_INFO("FORWARD! \n");
            cmd.linear.x = 0.6;
            cmd.angular.z = 0.0;
            moveDecision();
            break;
        case FOLLOW_WALL:
            ROS_INFO("Following the wall");
            if(min_front<0.9 && min_front>0.5)
            {
            cmd.angular.z = 1.5;
            cmd.linear.x = 0.4;
            }
            else {
            cmd.angular.z = 0.0;
            cmd.linear.x = 0.5;  
            }
            moveDecision();
            break;
        case FIND_THE_WALL:
            ROS_INFO("Searching for the wall");
            cmd.linear.x = 0.2;
            cmd.angular.z = -2.3;
            moveDecision();
            break;
        case TURN_LEFT:
            ROS_INFO("Turning left");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.9;
            moveDecision();
            break;
        case BACKWARD:
            ROS_INFO("Backword");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            moveDecision();
            break;
        case STOP:
            ROS_INFO("SUCCESS! \n");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
    }
    pub.publish(cmd);
}
void moveDecision()
{
    if(!s) {
        start = high_resolution_clock::now();
        s = true;
    }
    if((currentState == TURN_LEFT|| currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == BACKWARD) && min_right<d && min_front>d)
    {
        currentState = FOLLOW_WALL;
    }
    if((currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == BACKWARD) && min_right>d && ((min_front>d && min_front<9) || min_front>9))
    {
        currentState = FIND_THE_WALL;
    }
    if ((currentState ==  TURN_LEFT || currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == BACKWARD)
     && min_right>d && min_front>d && min_right>9 && min_front>9)
     {
           auto stop = high_resolution_clock::now();
            auto duration = duration_cast<seconds>(stop - start);
            std::cout << duration.count() << std::endl;
         currentState = STOP;
     }
     if((currentState == FORWARD || currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == TURN_LEFT || currentState == BACKWARD) && ((min_right<d && min_front<0.7*d) || (min_right>d && min_front<d)))
    {
        currentState = TURN_LEFT;
    }
    if((currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == TURN_LEFT || currentState == BACKWARD) && min_right<0.1 || min_front < 0.1)
    {
        currentState = BACKWARD;
    }
}



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    laser_msg=*scan_in;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    float right[30];
    float front[20];

    // right region
    for(int i = 0; i<30; i++ ) 
    {
        right[i] = laser_ranges[i];
    }



    // Front region
    for(int i = 80; i<100; i++)
    {
      front[i-80]=laser_ranges[i];
    }
    
    // Find the distance to the wall from right and front sides
    min_right = getMin(right,30);
    min_front = getMin(front,20);
    ROS_INFO("MIN_FRONT:  %f MIN_RIGHT: %f", min_front, min_right);
    StateMachine();
}

int main(int argc, char** argv)
{
   //Initializing ROS node
   ros::init(argc, argv, "wall_follower");		
   ros::NodeHandle n;
   sub = n.subscribe("/scan", 1, scanCallback);
   pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	  ros::Duration time_between_ros_wakeups(0.01);
	    while (ros::ok()) {
             
	        ros::spinOnce();
	        time_between_ros_wakeups.sleep();
	    }

    return 0;


}
