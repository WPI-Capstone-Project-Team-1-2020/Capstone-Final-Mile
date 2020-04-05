
#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <autonomy_msgs/GoalPose.h>
#include <autonomy_msgs/Takeoff.h>
#include <autonomy_msgs/Landing.h>
#include <autonomy_msgs/Status.h>
#include <autonomy_msgs/GoalReached.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

using namespace ros;

class GlobalPlanner
{
 public:
  GlobalPlanner(); 
  ~GlobalPlanner(); 
  bool init();
  void controlLoop(); //bool
  //void controlLoop(bool takeoff_status, bool land_status, bool local_reached); //bool
  void updateTakeoff(double altitude, bool reached);  //send takeoff 

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher goal_pose_pub_; 
  ros::Publisher takeoff_pub_;
  ros::Publisher landing_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber takeoff_status_sub_;
  ros::Subscriber landing_status_sub_;
  ros::Subscriber local_sub_;

  // Variables
  //double quad_pose_[3]; // = {0.0, 0.0, 0.0}; 
  //double quad_goal_pose_[3]; // = {0.0, 0.0, 0.0}; 
  bool takeoff_reached;
  bool land_reached;
  bool takeoff_status; //bool
  bool land_status; //bool
  bool local_status; //local_reached
  float odom_x; //double
  float odom_y; //double
  float end_x; //double
  float end_y; //double
  double altitude;
  int n;

  // Function prototypes
  void line(float end_x, float end_y); //linear trajectory //local_reached instead of local_status // double odom_x, double odom_y, bool local_status //double, double
  void updateLocalPlanner(double linear_x, double linear_y); // send goal to local planner
  void updateLanding(bool reached);  //send landing 
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg); // global planner input - was nav_msgs::Odometry
  void localMsgCallBack(const std_msgs::Bool::ConstPtr &msg); // receive goal status from local planner - std_msgs::Bool, autonomy_msgs::GoalReached
  void takeoffMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg);  //receive takeoff status
  void landingMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg);  //receive landing status
};
#endif // GLOBAL_PLANNER_HPP 
