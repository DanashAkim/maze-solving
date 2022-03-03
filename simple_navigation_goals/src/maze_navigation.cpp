#include <ros/ros.h>
#include <random>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h" 
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_srvs/Empty.h"

ros::Subscriber sub;
ros::Publisher pub;

double xGoal,yGoal;
float min_right,min_front;
std::vector<signed char> data;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
nav_msgs::MapMetaData info;
sensor_msgs::LaserScan laser_msg;
bool finish = false;


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

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    info = msg->info;
  	data = msg->data;
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) 
{
  laser_msg=*scan_in;
  std::vector<float> laser_ranges;
  laser_ranges = laser_msg.ranges;

  float right[20],front[20];

  // Right region
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

  //ROS_INFO("MIN_FRONT:  %f MIN_RIGHT: %f", min_front, min_right);

  if(min_right>9 && min_front>9) // Finish
  {
    finish = true;
    //ROS_INFO("%s", finish ? "true" : "false");
  }
}


class MyNode
{
public:
  MyNode() : ac("move_base", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(ros::Duration(3.0));
    ROS_INFO("Action server started, sending goal.");
  }

  void sendRandomGoal()
  {
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move to a random place
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    srand(time(NULL));

    double xMax = (info.height/2-1) * 0.05;

    double yMax = (info.width/2-1) * 0.05;
    double yMin = (info.width/2) * 0.05 * (-1);
    int i;

    do {
      double r = (double)rand() / (double)RAND_MAX;
      xGoal = xMax-r;
      yGoal = ((double) rand() / (double)RAND_MAX) * (yMax - yMin) + yMin;
      int x = (xGoal + 10)/0.05;
      int y = (yGoal + 10)/0.05;
      i = info.width * y + x;
    } while (data[i]==100);

  // double mapMax = 192 * 0.05;
  // double mapMin = -191 * 0.05;
  // int i;

  // do {
  //   //xGoal = ((double) rand() / (double)RAND_MAX) * (mapMax - mapMin) + mapMin;
    
  //   double r = (double)rand() / (double)RAND_MAX;
  //   xGoal = mapMax-r;
  //   yGoal = ((double) rand() / (double)RAND_MAX) * (mapMax - mapMin) + mapMin;
  //   int x = (xGoal + 10)/0.05;
  //   int y = (yGoal + 10)/0.05;
  //   i = info.width * y + x;
  // } while (data[i]==100);

    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.position.x = xGoal;
    goal.target_pose.pose.position.y = yGoal;
    ROS_INFO("Sending goal: x - %f, y - %f",xGoal,yGoal);

    ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                boost::bind(&MyNode::activeCb, this),
                boost::bind(&MyNode::feedbackCb, this, _1)); 
    ac.waitForResult();
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    ROS_INFO("Action STATE: %s",state.toString().c_str());
   if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Target is reached");
      ac.cancelAllGoals();
    }
    else
    {
    
      ROS_ERROR("move_base has failed");
      ROS_INFO("Action STATE: %s",state.toString().c_str());
      std_srvs::Empty emptymsg;
    ros::service::call("/move_base/clear_costmaps",emptymsg);
      ac.cancelAllGoals();
    }
  }

  void activeCb()
  {
    ROS_INFO("Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr&  feedback)
  {
      if(finish)
    {
      ROS_INFO("Finished");
      ac.cancelAllGoals();
    } 


  }

private:
  MoveBaseClient ac;
};


int main(int argc, char** argv){

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  sub = n.subscribe("/scan", 100, scanCallback);
  ros::Subscriber map_sub = n.subscribe("/map",1,mapCallBack);

    MyNode my_node;

  do {

    my_node.sendRandomGoal();
  } while(!finish);
  ROS_INFO("Congratulations!!!!!!!!!!");

  ros::waitForShutdown();
  
  return 0;
}
