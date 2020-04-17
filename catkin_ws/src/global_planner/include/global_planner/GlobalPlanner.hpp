
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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <autonomy_msgs/HospitalGoal.h>

using namespace ros;

class GlobalPlanner
{
 public:
  GlobalPlanner(); 
  ~GlobalPlanner(); 
  bool init();
  void controlLoop(float &end_x, float &end_y, bool land_reached, bool &count, bool &count2, ros::Time& begin); //bool
  //void controlLoop(bool takeoff_status, bool land_status, bool local_reached); //bool
  void updateTakeoff(double altitude, bool reached);  //send takeoff 
  void hopsitalCase(float &end_x, float &end_y, float x_hos_1, float y_hos_1, float x_hos_2, float y_hos_2, float x_hos_test_1, float y_hos_test_1, float x_hos_test_2, float y_hos_test_2); //choose hospital to fly to
  //was float

  //battery monitor
  void updateDiagnosticsBattery(const bool health, float max_flight); //updates battery monitor

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
  ros::Publisher diagnostics_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber takeoff_status_sub_;
  ros::Subscriber landing_status_sub_;
  ros::Subscriber local_sub_;
  ros::Subscriber hospital_goal_sub_;

  // Variables
  bool takeoff_reached;
  bool land_reached;
  bool takeoff_status; //bool
  bool land_status; //bool
  bool local_status; //local_reached
  float odom_x; //double
  float odom_y; //double
  double altitude;
  int n;
  bool health; //diagnostics
  int hospital_number; 
  float max_flight;

  //bool count;

  //Need to add params from file here
  // Hospital 1
  float x_hos_1; // x position for hospital 1
  float y_hos_1; // y position for hospital 1

  // Hospital 2
  float x_hos_2;  // x position for hospital 2
  float y_hos_2; // y position for hospital 2  

  // Hospital Test 1
  float x_hos_test_1; // x position for hospital test 1
  float y_hos_test_1; // y position for hospital test 1

  // Hospital Test 2
  float x_hos_test_2; // x position for hospital test 2
  float y_hos_test_2;  // y position for hospital test 2
 

  // Function prototypes

  // Diagnostics
  void updateDiagnostics(const bool health); //new for diagnostics
  void publishDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& statuses); //new for diagnostics
  void updateDiagnosticsDuration(const bool health, float flight_time_sec); //ros::Duration& flight_time); //this is new**

  // Update nodes
  void line(float &end_x, float &end_y); //linear trajectory //local_reached instead of local_status // double odom_x, double odom_y, bool local_status //double, double
  void updateLocalPlanner(double linear_x, double linear_y); // send goal to local planner
  void updateLanding(bool reached, float &end_x, float &end_y);  //send landing 
  
  // Callbacks
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg); // global planner input - was nav_msgs::Odometry
  void localMsgCallBack(const std_msgs::Bool::ConstPtr &msg); // receive goal status from local planner - std_msgs::Bool, autonomy_msgs::GoalReached
  void takeoffMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg);  //receive takeoff status
  void landingMsgCallBack(const autonomy_msgs::Status::ConstPtr &msg);  //receive landing status
  void hospitalMsgCallBack(const autonomy_msgs::HospitalGoal::ConstPtr &msg); //receive hospital goal
};
#endif // GLOBAL_PLANNER_HPP 
