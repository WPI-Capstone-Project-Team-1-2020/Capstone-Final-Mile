#ifndef CM_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP
#define CM_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP

// Component
#include "CostmapData.hpp"

// Ros
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>


// Standard
#include <memory>

namespace cm
{

// Forward declares
class CostmapConfig;

/// @brief Class for subscribing to topics
class TopicSubscriber
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the costmap
    TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<CostmapConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicSubscriber();

    /// @brief Accessor for cm data
    /// @return Costmap data
    const CostmapData& getCostmapData() const noexcept {return m_data;}

private:
    /// @brief Pointcloud callback
    /// @param msg Pointcloud message sent through IPC
    void onPointCloudReceived(const sensor_msgs::PointCloud2::ConstPtr& msg);


    /// @brief Local pose callback
    /// @param msg Local pose message sent through IPC
    void onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg);


    std::shared_ptr<CostmapConfig> m_cfg; ///< Config of the costmap

    ros::Subscriber m_pc_sub;                   ///< Pointcloud subscriber
    ros::Subscriber m_pose_sub;                 ///< Local pose subscriber

    CostmapData m_data; ///< Costmap Data
};    

} // namespace vi

#endif // VI_TOPIC_SUBSCRIBER_TOPIC_SUBSCRIBER_HPP