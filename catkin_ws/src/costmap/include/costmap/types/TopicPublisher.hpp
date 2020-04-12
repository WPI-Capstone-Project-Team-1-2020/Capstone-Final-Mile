#ifndef CM_TOPIC_PUBLISHER_HPP
#define CM_TOPIC_PUBLISHER_HPP

// Ros
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// Standard
#include <memory>

namespace cm
{

// Forward declares
class CostmapConfig;    

/// @brief Class for publishing to topics
class TopicPublisher
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the costmap
    TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<CostmapConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicPublisher();

    /// @brief Publishes command
    /// @param cm Costmap to publish
    void publishCostmap(const nav_msgs::OccupancyGrid::ConstPtr& cm);

    /// @brief Publishes diagnostics
    /// @param statuses Diagnostic statuses
    void publishDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& statuses);

private:

    std::shared_ptr<CostmapConfig>          m_cfg;         ///< Config of the costmap
    ros::Publisher                          m_costmap_pub; ///< Costmap publisher
    ros::Publisher                          m_diag_pub;    ///< Diagnostics publisher
};    

} // namespace cm

#endif // CM_TOPIC_PUBLISHER_HPP