
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <math.h>

#include "GlobalPlanner.hpp" 

#define pi 3.14159265;

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
  std::string diagnostics_topic_name = nh_.param<std::string>("diagnostics_topic_name", "");


  // initialize publishers
  goal_pose_pub_ = nh_.advertise<autonomy_msgs::GoalPose>(goal_pose_topic_name, 10);  //goal_pose_topic_name
  takeoff_pub_ = nh_.advertise<autonomy_msgs::Takeoff>(takeoff_topic_name, 10); //takeoff_topic_name
  landing_pub_ = nh_.advertise<autonomy_msgs::Landing>(landing_topic_name, 10); //landing_topic_name
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_name, 10); //diagnostics_topic_name //was DiagnosticStatus

  // initialize subscribers
  odom_sub_ = nh_.subscribe("ground_truth/state", 10, &GlobalPlanner::odomMsgCallBack, this); //"odom"
  takeoff_status_sub_ = nh_.subscribe("takeoff_status", 10, &GlobalPlanner::takeoffMsgCallBack, this); //"takeoff_status"
  landing_status_sub_ = nh_.subscribe("landing_status", 10, &GlobalPlanner::landingMsgCallBack, this); //"landing_status"
  local_sub_ = nh_.subscribe("local_planner/goal_reached", 10, &GlobalPlanner::localMsgCallBack, this);  //"local_status", then "goal_reached"
  hospital_goal_sub_ = nh_.subscribe("hospital_goal", 10, &GlobalPlanner::hospitalMsgCallBack, this); //"hospital_goal"

  return true;
}

void GlobalPlanner::hospitalMsgCallBack(const autonomy_msgs::HospitalGoal::ConstPtr& msg) //hospital goal callback
{
  //hospital message here
  hospital_number = msg->hospital_number;

  std::cout << "hospital number is " << hospital_number << std::endl;

}

void GlobalPlanner::updateDiagnostics(const bool health) //this is new**
{
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.level = health ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.name  = "Global Planner Node";

    array.status.push_back(status);

    diagnostics_pub_.publish(array); 
}


// Receive odometry data
void GlobalPlanner::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg) // was nav_msgs::Odometry
{

    odom_x = msg->pose.pose.position.x; //double //float
    odom_y = msg->pose.pose.position.y; //double //float

    //double odom_x = msg->x;
    //double odom_y = msg->y;

    //std::cout << "odom pos x is " << odom_x << " and y is " << odom_y << std::endl;

}



// Creating linear trajectory
void GlobalPlanner::line(float &end_x, float &end_y) //local_reached to local_status //included double odom_x, double odom_y, bool local_status //double
{
    autonomy_msgs::GoalPose goal_pose; 

    double n = 30; // magnitude in meters for each goal (magnitude of full line in sqrt(x_d^2 + y_d^2))
    
    float x_d = end_x - odom_x;  // delta x //double
    float y_d = end_y - odom_y; // delta y //double

    std::cout <<"odom_x is " << odom_x << " odom_y is " << odom_y << " x_d is " << x_d << " y_d is " << y_d << std::endl;


    if(x_d != 0 || y_d != 0) //false //local_reached
    {
      //wait
      //double x_d = end_x - odom_x;  // delta x
      //double y_d = end_y - odom_y; // delta y

      double angle = atan2(y_d,x_d); //double or float?

      std::cout << "angle is  " << angle << std::endl;

      double dist_to_goal = std::sqrt(std::pow(x_d, 2) + std::pow(y_d, 2));

      n = std::min(n, dist_to_goal);

      double x = odom_x + n*cos(angle); //*180/pi; //double or float? 
      double y = odom_y + n*sin(angle); //*180/pi; //double or float?

      goal_pose.x_m = x;
      goal_pose.y_m = y;
      goal_pose.speed_mps = 10;

      goal_pose_pub_.publish(goal_pose); 
      std::cout << "sent new local goal  " << goal_pose << std::endl;  //local_reached
    }
    else if(x_d == 0 || y_d == 0)  //true //local_reached
    {
      std::cout << " goal should be met" << std::endl;
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
void GlobalPlanner::updateLanding(bool reached, float &end_x, float &end_y)
{
  autonomy_msgs::Landing landing; 

  landing.goalReached = reached;
  landing.x_local = end_x;
  landing.y_local = end_y;

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
void GlobalPlanner::localMsgCallBack(const std_msgs::Bool::ConstPtr &msg) //std_msgs::Bool, autonomy_msgs::GoalReached
{
    //local_status = msg->goalReached; //local_reached

    local_status = msg->data; //local_reached

    std::cout << "local planner msg is " << local_status << std::endl; //local_reached
}

//float
void GlobalPlanner::hopsitalCase(float &end_x, float &end_y, float x_hos_1, float y_hos_1, float x_hos_2, float y_hos_2, float x_hos_test_1, float y_hos_test_1, float x_hos_test_2, float y_hos_test_2)
{

  //function for hospital selection
  //switch case 1-4
  switch (hospital_number)
  {
    case 1: //code to be executed if hospital number = 1
    //case 1: hospital 1
    end_x = x_hos_1;
    end_y = y_hos_1;
    std::cout << "hospital number is " << hospital_number << std::endl;
    std::cout << "x goal is " << end_x << ", y goal is " << end_y << std::endl;
    break;

    case 2: //code to be executed if hospital number = 2
    //case 2: hospital 2
    end_x = x_hos_2;
    end_y = y_hos_2;
    std::cout << "hospital number is " << hospital_number << std::endl;
    std::cout << "x goal is " << end_x << ", y goal is " << end_y << std::endl;
    break;

    case 3: //code to be executed if hospital number = 3
    //case 3: hospital test 1
    end_x = x_hos_test_1;
    end_y = y_hos_test_1;
    std::cout << "hospital number is " << hospital_number << std::endl;
    std::cout << "x goal is " << end_x << ", y goal is " << end_y << std::endl;
    break;

    case 4: //code to be excuted if hospital number = 4
    //case 4: hospital test 2
    end_x = x_hos_test_2;
    end_y = y_hos_test_2;
    std::cout << "hospital number is " << hospital_number << std::endl;
    std::cout << "x goal is " << end_x << ", y goal is " << end_y << std::endl;
    break;

    default: //code to be executed if hospital number is invalid
    std::cout << "hospital number is invalid, your value was " << hospital_number << ". Value must be 1-4" << std::endl;
    break;
  }

  //return(end_x, end_y);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
//void GlobalPlanner::controlLoop(bool takeoff_status, bool land_status, bool local_reached)  //bool
void GlobalPlanner::controlLoop(float &end_x, float &end_y, bool land_reached, bool &count, ros::Time& begin)  //bool
{

  //end_x = 287; //1292.0;
  //end_y = -1356; //205.5;

  GlobalPlanner::updateDiagnostics(true); 

  // Listen for takeoff goal reached
  if(takeoff_status != 1) //false = 0
  {
    std::cout << "waiting for takeoff to reach goal, takeoff_status is " << takeoff_status << std::endl;
  }
  else if(takeoff_status == 1) //true = 1
  {
    std::cout << "takeoff reached by global planner" << std::endl;
    //takeoff_reached = true;

    if(local_status != 1) //false = 0 //local_reached to local_status //was !=1
    {

      line(end_x, end_y); //linear trajectory  //local_reached to local_status //odom_x, odom_y, local_status
      
      //std::cout << "sending to local planner" << std::endl;
    }
    else if(local_status == 1) //true = 1  //local_reached to local_status //was ==1
    {
      //added if/else statement with count and only count this once (working code otherwise)
      if(count != 0)
      {
        count = false;
        std::cout << "count is " << count << std::endl;
        // Send landing command
        GlobalPlanner::updateLanding(land_reached, end_x, end_y);  //send landing 
        std::cout << "initiating land" << std::endl;

      }
      else if(count == 0)
      {
        //
        if(land_status != 1) //false = 0
        {
          std::cout << "waiting for landing goal to be reached, land_status is " << land_status << std::endl;
        }
        else if (land_status == 1) //true = 1
        {
          std::cout << "landing goal reached" << std::endl;
          ros::Time end = ros::Time::now(); //this might need to change
          std::cout << "end time is " << end << std::endl;

          ros::Duration flight_time = end - begin;
          std::cout << "duration of flight is " << flight_time << std::endl;
        }
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
  double altitude = 60;
  bool takeoff_reached = false; //false is 0
  bool land_reached = false; //false is 0
  bool local_reached = false; //false is 0
  bool count = true;
  float end_x;
  float end_y;

  // Hospital input
  //int hospital_number = 4; //eventually take out value
    //Need to add params from file here

  // Hospital 1
  float x_hos_1 = 287.0; // x position for hospital 1
  float y_hos_1 = -1356.0; // y position for hospital 1

  // Hospital 2
  float x_hos_2 = -219.0;  // x position for hospital 2
  float y_hos_2 = 1190.0; // y position for hospital 2  

  // Hospital Test 1
  float x_hos_test_1 = 220.0; // x position for hospital test 1
  float y_hos_test_1 = -1290.0; // y position for hospital test 1

  // Hospital Test 2
  float x_hos_test_2 = -19.0; // x position for hospital test 2
  float y_hos_test_2 = 900.0;  // y position for hospital test 2
  


  ros::init(argc, argv, "GlobalPlanner"); 
  GlobalPlanner GlobalPlanner;  

  ros::Rate loop_rate(10); //10 for 10x per second
  std::cout << "rate started" << std::endl;

  sleep(5);

  // Get hospital goal
  GlobalPlanner.hopsitalCase(end_x, end_y, x_hos_1, y_hos_1, x_hos_2, y_hos_2, x_hos_test_1, y_hos_test_1, x_hos_test_2, y_hos_test_2);

  // Send takeoff command
  GlobalPlanner.updateTakeoff(altitude, takeoff_reached);  //send takeoff 
  std::cout << "taking off" << std::endl;
  ros::Time begin = ros::Time::now();
  std::cout << "begin time is " << begin << std::endl;


  while (ros::ok())
  {
      // Get hospital goal
      GlobalPlanner.hopsitalCase(end_x, end_y, x_hos_1, y_hos_1, x_hos_2, y_hos_2, x_hos_test_1, y_hos_test_1, x_hos_test_2, y_hos_test_2);

      GlobalPlanner.controlLoop(end_x, end_y, land_reached, count, begin);  
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
