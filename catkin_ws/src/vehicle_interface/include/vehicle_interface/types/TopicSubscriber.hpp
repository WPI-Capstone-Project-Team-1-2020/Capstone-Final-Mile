#ifndef VI_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP
#define VI_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP

// Component
#include "VehicleInterfaceData.hpp"

// Ros
#include <autonomy_msgs/Status.h>
#include <autonomy_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Standard
#include <memory>

namespace vi
{

// Forward declares
class VehicleInterfaceConfig;

/// @brief Class for subscribing to topics
class TopicSubscriber
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the vehicle interface
    TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<VehicleInterfaceConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicSubscriber();

    /// @brief Accessor for vi data
    /// @return Vehicle interface data
    const VehicleInterfaceData& getVehicleInterfaceData() const noexcept {return m_data;}

private:
    /// @brief Goal reached callback
    /// @param msg `true` if goal has been reached
    void onGoalReachedReceived(const std_msgs::Bool::ConstPtr& msg);

    /// @brief Takeoff goal reached callback
    /// @param msg Goal reached message sent through IPC
    void onTakeoffGoalReachedReceived(const autonomy_msgs::Status::ConstPtr& msg);

    /// @brief Takeoff goal reached callback
    /// @param msg Goal reached message sent through IPC
    void onLandingGoalReachedReceived(const autonomy_msgs::Status::ConstPtr& msg);

    /// @brief Trajectory callback
    /// @param msg Trajectory message sent through IPC
    void onTrajectoryReceived(const autonomy_msgs::Trajectory::ConstPtr& msg);

    /// @brief Takeoff Land Command callback
    /// @param msg The command from either takeoff or landing nodes
    void onTakeoffLandReceived(const geometry_msgs::Twist::ConstPtr& msg);

    /// @brief Local pose callback
    /// @param msg Local pose message sent through IPC
    void onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg);


    std::shared_ptr<VehicleInterfaceConfig> m_cfg; ///< Config of the vehicle interface

    ros::Subscriber m_goal_reached_sub;         ///< Goal reached subscriber
    ros::Subscriber m_takeoff_reached_sub;      ///< Goal status of takeoff
    ros::Subscriber m_landing_reached_sub;      ///< Goal status of land
    ros::Subscriber m_traj_sub;                 ///< Goal pose subscriber
    ros::Subscriber m_takeoff_land_sub;         ///< Takeoff/landing cmd subscriber
    ros::Subscriber m_pose_sub;                 ///< Local pose subscriber

    VehicleInterfaceData m_data; ///< vehicle interface Data
};    

} // namespace vi

#endif // VI_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP