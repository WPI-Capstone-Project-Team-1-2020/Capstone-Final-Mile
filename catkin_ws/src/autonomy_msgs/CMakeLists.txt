cmake_minimum_required(VERSION 2.8.12)
project(autonomy_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation  
  std_msgs
)

add_message_files(
  FILES
  GoalPose.msg
  Trajectory.msg
  Takeoff.msg
  Landing.msg
  GoalReached.msg
  Status.msg
  HospitalGoal.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

