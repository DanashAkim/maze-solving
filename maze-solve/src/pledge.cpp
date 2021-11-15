#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include "tf/transform_listener.h" 
#include <nav_msgs/Odometry.h>
#include  <stdlib.h>

ros::Subscriber odom;
ros::Publisher pub;
ros::Subscriber sub;

geometry_msgs::Twist cmd;
sensor_msgs::LaserScan laser_msg;
float min_right, min_front;
double pi = 3.14159;
float d = 0.8; // optimal distance to the wall
double roll, pitch, yaw = 0.0, yaw_original,yaw_prev,yaw_diff;
double heading_current, yaw_sum = 0.0; 
bool first=true;

void moveDecision();

//Behaviour of the robot
typedef enum _ROBOT_MOVEMENT {
    FIND_THE_WALL,
    TURN_LEFT,
    FOLLOW_WALL,
    STOP,
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

void odometry(const nav_msgs::Odometry::ConstPtr& msg)
{

 tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    yaw_prev = yaw;

    m.getRPY(roll, pitch, yaw);


    if (first) 
    {
        yaw_original = yaw;
        first = false;
    }
    heading_current = abs(yaw - yaw_original);
    yaw_diff = yaw-yaw_prev;
    
    if(yaw_diff > pi)
    {
        yaw_diff = yaw_diff - 2*pi;
    }
         
    if(yaw_diff < -pi) 
    {
        yaw_diff = yaw_diff + 2*pi;
    }
    yaw_sum = yaw_sum + yaw_diff;
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
            cmd.linear.x = 0.6;  
            }
            moveDecision();
            break;
        case FIND_THE_WALL:
            ROS_INFO("Searching for the wall");
            cmd.linear.x = 0.5;
            cmd.angular.z = -1.5;
            moveDecision();
            break;
        case TURN_LEFT:
            ROS_INFO("Turning left");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.9;
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
    if((currentState == TURN_LEFT|| currentState == FOLLOW_WALL || currentState == FIND_THE_WALL) && min_right<d && min_front>d)
    {
        currentState = FOLLOW_WALL;
    }
    if((currentState == FOLLOW_WALL || currentState == FIND_THE_WALL) && min_right>d && min_front>d && !(heading_current < 0.08 && abs(yaw_sum) < 0.08))
    {
        currentState = FIND_THE_WALL;
    }
    if((currentState ==  TURN_LEFT || currentState == FOLLOW_WALL || currentState == FORWARD)
     && min_right>d && min_front>d && heading_current < 0.08 && abs(yaw_sum) < 0.08 && !(min_right>9 && min_front>9)) 
     {
         currentState = FORWARD;
     }
    if ((currentState == FORWARD || currentState ==  TURN_LEFT || currentState == FOLLOW_WALL || currentState == FIND_THE_WALL)
     && min_right>d && min_front>d && heading_current < 0.08 && abs(yaw_sum) < 0.08 && min_right>9 && min_front>9)
     {
         currentState = STOP;
     }
     if((currentState == FORWARD || currentState == FOLLOW_WALL || currentState == FIND_THE_WALL || currentState == TURN_LEFT) && ((min_right<d && min_front<0.8*d) || (min_right>d && min_front<d)))
    {
        currentState = TURN_LEFT;
    }
}



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    laser_msg=*scan_in;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    float right[20];
    float front[20];

    // right region
    for (int i = 0; i<20; i++)
    {
      right[i] = laser_ranges[i];
    }

    // Front region
    for(int i = 80; i<100; i++)
    {
      front[i-80]=laser_ranges[i];
    }
    
    // Find the distance to the wall from right and front sides
    min_right = getMin(right,20);
    min_front = getMin(front,20);

    // ROS_INFO("MIN_FRONT:  %f MIN_RIGHT: %f", min_front, min_right);
    ROS_INFO("Yaw sum: %f", yaw_sum);
    ROS_INFO("Turn: %f", heading_current);
    
    StateMachine();
}

int main(int argc, char** argv)
{
   //Initializing ROS node
   ros::init(argc, argv, "pledge");		
   ros::NodeHandle n;
   odom = n.subscribe("/odom", 1,odometry);
   sub = n.subscribe("/scan", 1, scanCallback);
   pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	  ros::Duration time_between_ros_wakeups(0.01);
	    while (ros::ok()) {
             
	        ros::spinOnce();
	        time_between_ros_wakeups.sleep();
	    }

    return 0;


}
