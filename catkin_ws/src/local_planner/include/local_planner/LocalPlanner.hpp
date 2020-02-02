#ifndef PLANNING_LOCAL_PLANNER_HPP
#define PLANNING_LOCAL_PLANNER_HPP

// Component
#include "GraphNode.hpp"

// Ros
#include <ros/ros.h>

// Standard
#include <memory>

namespace local_planner
{

// Forward Declarations
class LocalPlannerConfig;
class TopicSubscriber;

/// @brief Class for planning local trajectories
class LocalPlanner
{
public:
    /// @brief Default Constructor
    /// @param nh nodehandle
    /// @param pnh private nodehandle
    LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /// @brief Default destructor
    ~LocalPlanner();

private:

    std::shared_ptr<LocalPlannerConfig> m_cfg;       ///< Configuration of local planner
    std::unique_ptr<TopicSubscriber>    m_topic_sub; ///< Topic Subscriber

};
} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_HPP