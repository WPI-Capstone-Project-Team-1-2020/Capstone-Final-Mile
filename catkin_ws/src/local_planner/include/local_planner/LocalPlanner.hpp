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
class AStar;
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
    ~LocalPlanner() = default;

private:
    /// @brief Main driving function of the local planner
    /// @param event Timer event
    void update(const ros::TimerEvent& event);

    std::shared_ptr<LocalPlannerConfig> m_cfg;       ///< Configuration of local planner
    std::unique_ptr<TopicSubscriber>    m_topic_sub; ///< Topic Subscriber
    ros::Timer                          m_timer;     ///< Timer to drive update cycles

    std::unique_ptr<AStar>              m_solver;    ///< A* Solver to plan paths

};
} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_HPP