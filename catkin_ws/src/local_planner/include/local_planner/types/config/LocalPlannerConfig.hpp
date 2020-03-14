#ifndef PLANNING_LOCAL_PLANNER_CONFIG_HPP
#define PLANNING_LOCAL_PLANNER_CONFIG_HPP

// Component
#include "GraphNodeToleranceConfig.hpp"
// Libraries
#include <boost/cstdfloat.hpp>

// Ros
#include <ros/ros.h>

// Standard
#include <cstdint>
#include <string>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for local planner
class LocalPlannerConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle used to snipe params
    LocalPlannerConfig(ros::NodeHandle& pnh) : 
        m_node_cfg{GraphNodeToleranceConfig(pnh)}
    {
        pnh.getParam("goal_topic",                  m_goal_topic);
        pnh.getParam("local_pose_topic",            m_local_pose_topic);
        pnh.getParam("update_rate_hz",              m_update_rate_hz);
    }

    /// @brief Default destructor for forward declares
    LocalPlannerConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    const std::string&              getGoalTopic()                 const noexcept {return m_goal_topic;}
    const std::string&              getLocalPoseTopic()            const noexcept {return m_local_pose_topic;}
    float64_t                       getUpdateRateHz()              const noexcept {return m_update_rate_hz;}
    const GraphNodeToleranceConfig& getGraphNodeToleranceConfig()  const noexcept {return m_node_cfg;}


private:
    /// @brief I/O Topics
    /// @{
    std::string m_goal_topic{""};       ///< Goal topic
    std::string m_local_pose_topic{""}; ///< Local pose topic    
    /// @}

    /// @brief Performance
    /// @{
    float64_t m_update_rate_hz{10.0}; ///< Update rate in hz
    /// @}

    /// @brief Tuning Parameters
    /// @{
    GraphNodeToleranceConfig m_node_cfg; ///< Graph node tolerance config
    /// @}
};

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_CONFIG_HPP
