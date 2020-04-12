#ifndef CM_COSTMAP_HPP
#define CM_COSTMAP_HPP

// Ros
#include <ros/ros.h>

// Standard
#include <memory>

namespace cm
{

// Forward Declarations
class CostmapConfig;
class GridBuilder;
class TopicPublisher;
class TopicSubscriber;

/// @brief Class for Costmap to publish commands to vehicle
class Costmap
{
public:
    /// @brief Default Constructor
    /// @param nh nodehandle
    /// @param pnh private nodehandle
    Costmap(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /// @brief Default destructor    
    ~Costmap();

private:
    /// @brief Main dricmng function of the Costmap
    /// @param event Timer event
    void update(const ros::TimerEvent& event);

    /// @brief Updates diagnostics
    /// @param health `true` if healthy
    void updateDiagnostics(const bool health);
    
    std::shared_ptr<CostmapConfig>          m_cfg;         ///< Configuration of Costmap    
    std::unique_ptr<TopicSubscriber>        m_topic_sub;   ///< Topic Subscriber
    std::unique_ptr<TopicPublisher>         m_topic_pub;   ///< Topic Publisher
    std::unique_ptr<GridBuilder>            m_builder;     ///< Occupancy grid builder
    ros::Timer                              m_timer;       ///< Timer to drive update cycles
};

} // namespace cm

#endif // CM_COSTMAP_HPP