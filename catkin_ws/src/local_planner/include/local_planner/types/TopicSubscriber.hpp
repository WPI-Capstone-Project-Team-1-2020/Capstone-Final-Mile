#ifndef PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP
#define PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP

// Component
#include "LocalPlannerData.hpp"

// Ros
#include <autonomy_msgs/GoalPose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Standard
#include <memory>

// Forward declares
class LocalPlannerConfig;

namespace local_planner
{

/// @brief Class for subscribing to topics
class TopicSubscriber
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the local planner
    TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicSubscriber();

    /// @brief Accessor for local planner data
    /// @return Local planner data
    const LocalPlannerData& getLocalPlannerData() const noexcept {return m_data;}

private:
    /// @brief Goal pose callback
    /// @param msg Goal pose message sent through IPC
    void onGoalPoseReceived(const autonomy_msgs::GoalPose::ConstPtr& msg);

    /// @brief Local pose callback
    /// @param msg Local pose message sent through IPC
    void onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg);


    std::shared_ptr<LocalPlannerConfig> m_cfg; ///< Config of the local planner

    ros::Subscriber m_goal_sub; ///< Goal pose subscriber
    ros::Subscriber m_pose_sub; ///< Local pose subscriber

    LocalPlannerData m_data; ///< Local Planner Data
};    

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP