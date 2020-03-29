
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/callback_queue.h>

#include "GlobalPlanner.hpp" 

using namespace ros;

GlobalPlanner::GlobalPlanner() 
  : nh_priv_("~")
{
  //Init gazebo ros global planner node
  ROS_INFO("Global Planner Simulation Node Init");
  //ROS_ASSERT(init());
  init();
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
  std::string takeoff_status_topic_name = nh_.param<std::string>("takeoff_status_topic_name", "");
  std::string landing_status_topic_name = nh_.param<std::string>("landing_status_topic_name", "");
  std::string local_status_topic_name = nh_.param<std::string>("local_status_topic_name", "");
  std::string odom_topic_name = nh_.param<std::string>("odom_topic_name", "");

  // initialize variables
  //double quad_pose[3] = {-1231.0, -285.6, 13.5}; 
  //double quad_goal_pose[3] = {1292, 205.5, 8};  

  // initialize publishers
  goal_pose_pub_ = nh_.advertise<autonomy_msgs::GoalPose>("goal_pose_topic_name", 10);  //goal_pose_topic_name
  takeoff_pub_ = nh_.advertise<autonomy_msgs::Takeoff>("takeoff_topic_name", 10); //takeoff_topic_name
  landing_pub_ = nh_.advertise<autonomy_msgs::Landing>("landing_topic_name", 10); //landing_topic_name

  // initialize subscribers
  odom_sub_ = nh_.subscribe("odom", 1, &GlobalPlanner::odomMsgCallBack, this);
  takeoff_status_sub_ = nh_.subscribe("takeoff_status", 1, &GlobalPlanner::takeoffMsgCallBack, this);
  landing_status_sub_ = nh_.subscribe("landing_status", 1, &GlobalPlanner::landingMsgCallBack, this); 
  local_sub_ = nh_.subscribe("local_status", 1, &GlobalPlanner::localMsgCallBack, this); 

  return true;
}


// Receive odometry data
void GlobalPlanner::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{

    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;

    std::cout << "odom pos x is " << odom_x << " and y is " << odom_y << std::endl;

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

void GlobalPlanner::initLine(double start_x, double end_x, double start_y, double end_y, int n) //added this
{
  autonomy_msgs::GoalPose goal_pose;

  double m = (start_y-end_y)/(start_x-end_x);
  double b = start_y - m*start_x;

  goal_pose.speed_mps = 10;

  std::vector<double>array = GlobalPlanner::linspace(start_x, end_x, start_y, end_y, n); //array = // std::vector<double>linspace()

  double y = m*array[0] + b;

  std::cout << "y is " << y << " and x is  " << array[0] << std::endl;

  goal_pose.x_m = array[0];
  goal_pose.y_m = y;

  goal_pose_pub_.publish(goal_pose); 
}

// Creating linear trajectory
void GlobalPlanner::line(double start_x, double end_x, double start_y, double end_y, bool local_reached)
{
    autonomy_msgs::GoalPose goal_pose; 

    //std::vector<double> linspace(double start_x, double end_x, double start_y, double end_y, int n);

    int n = 50;
    double m = (start_y-end_y)/(start_x-end_x);
    double b = start_y - m*start_x;

    goal_pose.speed_mps = 10;

    std::vector<double>array = GlobalPlanner::linspace(start_x, end_x, start_y, end_y, n); //array = // std::vector<double>linspace()

    //double y = m*array[1] + b;

    //std::cout << "y is " << y << " and x is  " << array[1] << std::endl;

    //goal_pose.x_m = array[1];
    //goal_pose.y_m = y;

    //goal_pose_pub_.publish(goal_pose);

    //std::cout << " in line function and goal_pose is" << goal_pose << std::endl;

    for(int i=1; i<n; i++)
    {
      if(local_reached != 1) //false
      {
        //wait
        std::cout << "waiting for local goal to be reached, local_reache is  " << local_reached << std::endl;
      }
      else if(local_reached == 1)  //true
      {
        double y = m*array[i] + b;

        std::cout << "y is " << y << " and x is  " << array[i] << std::endl;

        goal_pose.x_m = array[i];
        goal_pose.y_m = y;

        goal_pose_pub_.publish(goal_pose);

        std::cout << " in line function and goal_pose is" << goal_pose << std::endl;

        //add in send local goal reached
        local_reached = false;
      }
        //local_reached = false; //this was added
    }
    

}

// Send takeoff messages
void GlobalPlanner::updateTakeoff(double altitude, bool reached) 
{
  autonomy_msgs::Takeoff takeoff;

  std::cout << "inside update takeoff function" << std::endl;

  takeoff.height = altitude;
  takeoff.goalReached = reached;

  std::cout << "takeoff goal is " << reached << "altitude given is " << altitude << std::endl;
  //std::cout << "altitude given is " << altitude << std::endl;

  takeoff_pub_.publish(takeoff);
}

// Receive takeoff messages
void GlobalPlanner::takeoffMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg)
{
    takeoff_status = msg->status;

    std::cout << "inside takeoff callback function, message from takeoff is " << takeoff_status << std::endl;
    //std::cout << "message from takeoff is " << takeoff_status << std::endl;
}

// Send landing messages
void GlobalPlanner::updateLanding(bool reached)
{
  autonomy_msgs::Landing landing; 

  landing.goalReached = reached;

  landing_pub_.publish(landing);

  std::cout << "inside landing and publishing" << landing << std::endl;
}

// Receive landing messages
void GlobalPlanner::landingMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg)
{
    land_status = msg->status;

    std::cout << "landing callback is msg is " << land_status << std::endl;
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
    local_reached = msg->goalReached;

    std::cout << "local planner msg is " << local_reached << std::endl;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
void GlobalPlanner::controlLoop(bool takeoff_reached, bool land_reached, bool local_reached)  //bool
{

  // Initial parameters  
  //double altitude = 18.5;
  //bool takeoff_reached = false;
  //bool local_reached = false;
  //bool land_reached = false;

  start_x = -1231.0;
  end_x = 1292.0;
  start_y = -285.6;
  end_y = 205.5;


  // Listen for takeoff goal reached
  if(takeoff_status != 1) //false = 0
  {
    std::cout << "waiting for takeoff to reach goal" << std::endl;
  }
  else if(takeoff_status == 1) //true = 1
  {
    std::cout << "takeoff reached by global planner" << std::endl;

    if(local_reached != 1) //false = 0
    {
      line(start_x, end_x, start_y, end_y, local_reached); //linear trajectory
      
      std::cout << "sending to local planner" << std::endl;
    }
    else if(local_reached == 1) //true = 1
    {
      if(land_status != 1) //false = 0
      {
        // Send landing command
        GlobalPlanner::updateLanding(land_reached);  //send landing 
        std::cout << "initiating land" << std::endl;
      }
      else if (land_status == 1) //true = 1
      {
        std::cout << "landing goal reached" << std::endl;
      }
      
    }
    
  }


  return; //true
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  std::cout << "global planner started" << std::endl;

  //Initialize parameters
  double altitude = 30;
  bool takeoff_reached = false; //false is 0
  bool land_reached = false; //false is 0
  bool local_reached = false; //false is 0
  int n = 50;
  double start_x = -1231.0;
  double end_x = 1292.0;
  double start_y = -285.6;
  double end_y = 205.5;

  //ros::CallbackQueue my_queue;

  ros::init(argc, argv, "GlobalPlanner"); 
  GlobalPlanner GlobalPlanner;  

  ros::Rate loop_rate(10); //125 per second - changed to 5 for 5x per second
  std::cout << "rate started" << std::endl;

  // Send takeoff command
  GlobalPlanner.updateTakeoff(altitude, takeoff_reached);  //send takeoff 
  std::cout << "taking off" << std::endl;

  GlobalPlanner.initLine(start_x, end_x, start_y, end_y, n);

  while (ros::ok())
  {
    GlobalPlanner.controlLoop(takeoff_reached, land_reached, local_reached);  
    //my_queue.callAvailable(ros::WallDuration(5));
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
