#ifndef VI_TOPIC_PUBLISHER_HPP
#define VI_TOPIC_PUBLISHER_HPP

// Ros
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Standard
#include <memory>

namespace vi
{

// Forward declares
class VehicleInterfaceConfig;    

/// @brief Class for publishing to topics
class TopicPublisher
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the vehicle interface
    TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<VehicleInterfaceConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicPublisher();

    /// @brief Publishes command
    /// @param cmd Command to publish
    void publishCommand(const geometry_msgs::Twist::ConstPtr& cmd);

    /// @brief Publishes diagnostics
    /// @param statuses Diagnostic statuses
    void publishDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& statuses);

private:

    std::shared_ptr<VehicleInterfaceConfig> m_cfg;      ///< Config of the vehicle interface
    ros::Publisher                          m_traj_pub; ///< Trajectory publisher
    ros::Publisher                          m_diag_pub; ///< Diagnostics publisher
};    

} // namespace vi

#endif // VI_TOPIC_PUBLISHER_HPP