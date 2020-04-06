#ifndef PLANNING_LOCAL_PLANNER_CONFIG_HPP
#define PLANNING_LOCAL_PLANNER_CONFIG_HPP

// Component
#include "GraphNodeToleranceConfig.hpp"
#include "TrajectoryConfig.hpp"

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
        m_node_cfg{GraphNodeToleranceConfig(pnh)},
        m_traj_cfg{TrajectoryConfig(pnh)}
    {
        pnh.getParam("costmap_topic",               m_costmap_topic);
        pnh.getParam("goal_topic",                  m_goal_topic);
        pnh.getParam("goal_reached_topic",          m_goal_reached_topic);
        pnh.getParam("local_pose_topic",            m_local_pose_topic);
        pnh.getParam("local_traj_topic",            m_traj_topic);
        pnh.getParam("path_topic",                  m_path_topic);
        pnh.getParam("update_rate_hz",              m_update_rate_hz);
        pnh.getParam("goal_reached_tol_m",          m_goal_tol_m);
        pnh.getParam("max_goal_delta_m",            m_max_goal_delta_m);
        pnh.getParam("search_timeout_ms",           m_timeout_ms);
        pnh.getParam("lethal_cost_probability",     m_lethal_prob);
        pnh.getParam("non_lethal_cost_mult",        m_non_lethal_cost_mult);
    }

    /// @brief Default destructor for forward declares
    LocalPlannerConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{      
    const std::string&              getCostmapTopic()                   const noexcept {return m_costmap_topic;}
    const std::string&              getGoalTopic()                      const noexcept {return m_goal_topic;}
    const std::string&              getGoalReachedTopic()               const noexcept {return m_goal_reached_topic;}
    const std::string&              getLocalPoseTopic()                 const noexcept {return m_local_pose_topic;}
    const std::string&              getTrajectoryTopic()                const noexcept {return m_traj_topic;}
    const std::string&              getPathTopic()                      const noexcept {return m_path_topic;}
    float64_t                       getUpdateRateHz()                   const noexcept {return m_update_rate_hz;}
    float64_t                       getGoalReachedTolerance()           const noexcept {return m_goal_tol_m;}
    float64_t                       getMaxGoalDeltaM()                  const noexcept {return m_max_goal_delta_m;}
    std::int32_t                    getSearchTimeoutMs()                const noexcept {return m_timeout_ms;}
    std::int32_t                    getLethalProbability()              const noexcept {return m_lethal_prob;}
    float64_t                       getNonLethalCostMult()              const noexcept {return m_non_lethal_cost_mult;}
    const GraphNodeToleranceConfig& getGraphNodeToleranceConfig()       const noexcept {return m_node_cfg;}
    const TrajectoryConfig&         getTrajectoryConfig()               const noexcept {return m_traj_cfg;}


private:
    /// @brief I/O Topics
    /// @{
    std::string m_costmap_topic{""};        ///< Costmap topic
    std::string m_goal_topic{""};           ///< Goal topic
    std::string m_goal_reached_topic{""};   ///< Goal reached topic
    std::string m_local_pose_topic{""};     ///< Local pose topic  
    std::string m_traj_topic{""};           ///< Trajectory publish topic
    std::string m_path_topic{""};           ///< Path publish topic
    /// @}

    /// @brief Performance
    /// @{
    float64_t    m_update_rate_hz{10.0};   ///< Update rate in hz
    float64_t    m_goal_tol_m{0.5};        ///< Tolerance when checking for goal (outside of solvers)
    float64_t    m_max_goal_delta_m{40.0}; ///< Max distance a goal can be from current pose
    std::int32_t m_timeout_ms{150};        ///< Timeout in ms
    /// @}

    /// @brief Collision parameters
    /// @{
    std::int32_t m_lethal_prob{90};                    ///< Probability of space being occupied that is unrecoverable from
    float64_t    m_non_lethal_cost_mult{1000000.0};    ///< Multiplier on cost for non-lethal collissions
    /// @}

    /// @brief Tuning Parameters
    /// @{
    GraphNodeToleranceConfig m_node_cfg;             ///< Graph node tolerance config
    TrajectoryConfig         m_traj_cfg;             ///< Trajectory config
    /// @}
};

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_CONFIG_HPP
