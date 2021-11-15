
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include "tf/transform_listener.h" 
ros::Publisher pub;
ros::Subscriber sub;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist cmd;
float min_right;
float min_front;

//Behaviour of the robot
typedef enum _ROBOT_MOVEMENT {
    FIND_THE_WALL,
    TURN_LEFT,
    FOLLOW_WALL,
    BACKWARD,
    STOP

} ROBOT_MOVEMENT;

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
    // Movement of the robot
void move(const ROBOT_MOVEMENT move_type)
{

    
     if (move_type == FIND_THE_WALL)
    {

        ROS_INFO("Searching for the wall");
        cmd.linear.x = 0.5;
        cmd.angular.z = -1.5;
        
    }

    else if (move_type == TURN_LEFT)
    {
        ROS_INFO("Turning left");
        
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.9;
    }

    else if (move_type == FOLLOW_WALL)
    {   
        ROS_INFO("Following the wall");

        if(min_front<0.9 && min_front>0.5)
        {
        cmd.angular.z = 1.5;
        cmd.linear.x = 0.8;
        }
        else {
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.8;  
        }
    }
      else if (move_type == BACKWARD) {
        ROS_INFO("I'm going back! \n");
        cmd.linear.x = -0.3;
        cmd.angular.z = 0.0;
    }
        else if (move_type == STOP) {
        ROS_INFO("SUCCESS! \n");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
       
    
    pub.publish(cmd);
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
    ROS_INFO("MIN_FRONT:  %f MIN_RIGHT: %f", min_front, min_right);

    float d = 0.6; // optimal distance to the wall

    cmd.angular.z = 0.0;
    cmd.linear.x = 0.0;



        if (min_right>d && min_right<9 && ((min_front>d && min_front<9) || min_front>9))//nothing
        {
            move(FIND_THE_WALL);
        }
        else if (min_right<d && min_front>d) //right
        {
            move(FOLLOW_WALL);
        }
        else if (min_right<d && min_front<0.8*d) //corner (front+right)
        {
            move(TURN_LEFT);
        }
        else if (min_right>d && min_front<d) //front
        {
            move(TURN_LEFT);
        }
        else if (min_right<0.1 || min_front < 0.1) // Crashed
        {
            move(BACKWARD);
        }
        else if (min_right>9 && min_front>9) // Finish
        {
            move(STOP);
        }

}



int main(int argc, char** argv)
{
   //Initializing ROS node
   ros::init(argc, argv, "wall_follower");		
   ros::NodeHandle n;

   sub = n.subscribe("/scan", 100, scanCallback);
   pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	  ros::Duration time_between_ros_wakeups(0.001);
	    while (ros::ok()) {
	        ros::spinOnce();
	        time_between_ros_wakeups.sleep();
	    }

    return 0;


}
