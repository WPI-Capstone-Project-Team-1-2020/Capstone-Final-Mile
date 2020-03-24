
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/rate.h>

#include "GlobalPlanner.hpp" 

using namespace ros;

GlobalPlanner::GlobalPlanner() 
  : nh_priv_("~")
{
  //Init gazebo ros global planner node
  ROS_INFO("Global Planner Simulation Node Init");
  ROS_ASSERT(init());
}

GlobalPlanner::~GlobalPlanner() 
{
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool GlobalPlanner::init() 
{
  // initialize ROS parameter
  std::string goal_pose_topic_name = nh_.param<std::string>("goal_pose_topic_name", "");
  std::string takeoff_topic_name = nh_.param<std::string>("takeoff_topic_name", "");
  std::string landing_topic_name = nh_.param<std::string>("landing_topic_name", "");
  std::string local_status_topic_name = nh_.param<std::string>("local_status_topic_name", "");
  std::string odom_topic_name = nh_.param<std::string>("odom_topic_name", "");

  // initialize variables
  //double quad_pose[3] = {-1231.0, -285.6, 13.5}; 
  //double quad_goal_pose[3] = {1292, 205.5, 8};  

  // initialize publishers
  goal_pose_pub_   = nh_.advertise<autonomy_msgs::GoalPose>(goal_pose_topic_name, 10); 
  takeoff_pub_   = nh_.advertise<autonomy_msgs::Takeoff>(takeoff_topic_name, 10); 
  landing_pub_   = nh_.advertise<autonomy_msgs::Landing>(landing_topic_name, 10); 

  // initialize subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &GlobalPlanner::odomMsgCallBack, this);
  takeoff_sub_ = nh_.subscribe("takeoff", 10, &GlobalPlanner::takeoffMsgCallBack, this);
  landing_sub_ = nh_.subscribe("landing", 10, &GlobalPlanner::landingMsgCallBack, this); 
  local_sub_ = nh_.subscribe("local_status", 10, &GlobalPlanner::localMsgCallBack, this); 

  return true;
}


// Receive odometry data
void GlobalPlanner::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{

    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;

}

// can I have nested function??
std::vector<double> GlobalPlanner::linspace(double start_x, double end_x, double start_y, double end_y, int n)
{
    std::vector<double> array;
    double step = (end_x - start_x)/(n-1);

    while(start_x <= end_x) 
    {
        array.push_back(start_x);
        start_x += step;
    }
    return array;
}

// Creating linear trajectory
void GlobalPlanner::line(double start_x, double start_y, double end_x, double end_y)
{
    autonomy_msgs::GoalPose goal_pose; 

    //std::vector<double> linspace(double start_x, double end_x, double start_y, double end_y);

    int n = 50;
    double m = (start_y-end_y)/(start_x-end_x);
    double b = start_y - m*start_x;

    std::vector<double>array = GlobalPlanner::linspace(start_x, end_x, start_y, end_y, n); //array = // std::vector<double>linspace()

    for(int i=0; i<n; i++)
    {
        double y = m*array[i] + b;

        goal_pose.x_m = array[i];
        goal_pose.y_m = y;

        goal_pose_pub_.publish(goal_pose);

    }

}

// Send takeoff messages
void GlobalPlanner::updateTakeoff(double altitude, bool reached) 
{
  autonomy_msgs::Takeoff takeoff;

  takeoff.height = altitude;
  takeoff.goalReached = reached;

  takeoff_pub_.publish(takeoff);
}

// Receive takeoff messages
void GlobalPlanner::takeoffMsgCallBack(const autonomy_msgs::Takeoff::ConstPtr &msg)
{
    bool takeoff_reached = msg->goalReached;
}

// Send landing messages
void GlobalPlanner::updateLanding(bool reached)
{
  autonomy_msgs::Landing landing; 

  landing.goalReached = reached;

  landing_pub_.publish(landing);
}

// Receive landing messages
void GlobalPlanner::landingMsgCallBack(const autonomy_msgs::Landing::ConstPtr &msg)
{
    bool land_reached = msg->goalReached;
}


// Send Local Planner messages
void GlobalPlanner::updateLocalPlanner(double linear_x, double linear_y) 
{
  autonomy_msgs::GoalPose goal_pose; 

  goal_pose.x_m = linear_x;
  goal_pose.y_m = linear_y;

  goal_pose_pub_.publish(goal_pose);
}


// Receive Local Planner messages
void GlobalPlanner::localMsgCallBack(const autonomy_msgs::GoalReached::ConstPtr &msg)
{
    bool local_reached = msg->goalReached;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool GlobalPlanner::controlLoop(bool takeoff_reached, bool land_reached, bool local_reached)  
{

  // Initial parameters  
  //double altitude = 18.5;
  //bool takeoff_reached = false;
  //bool local_reached = false;
  //bool land_reached = false;

  double start_x = -1231.0;
  double end_x = 1292.0;
  double start_y = -285.6;
  double end_y = 205.5;


  // Listen for takeoff goal reached
  if(takeoff_reached = true) 
  {
    if(local_reached = false)
    {
      line(start_x, start_y, end_x, end_y); //linear trajectory
    }
    else
    {
      if(land_reached = false)
      {
        // Send landing command
        GlobalPlanner::updateLanding(land_reached);  //send landing 
      }
      else
      {
        
      }
      
    }
    
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  //Initialize parameters
  double altitude = 18.5;
  bool takeoff_reached = false;
  bool land_reached = false;
  bool local_reached = false;

  ros::init(argc, argv, "GlobalPlanner"); 
  GlobalPlanner GlobalPlanner;  

  ros::Rate loop_rate(125);

  // Send takeoff command
  GlobalPlanner.updateTakeoff(altitude, takeoff_reached);  //send takeoff 

  while (ros::ok())
  {
    GlobalPlanner.controlLoop(takeoff_reached, land_reached, local_reached);  
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
