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
    LocalPlannerConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("goal_topic",                  m_goal_topic);
        pnh.getParam("local_pose_topic",            m_local_pose_topic);
        pnh.getParam("update_rate_hz",              m_update_rate_hz);
        pnh.getParam("time_step_ms",                m_time_step_ms);
        pnh.getParam("velocity_discretization_mps", m_velocity_discretization_mps);
        pnh.getParam("yaw_rate_discretization_mps", m_yaw_rate_discretization_mps);
        pnh.getParam("max_vel_mps",                 m_max_vel_mps);
        pnh.getParam("max_lateral_accel_mpss",      m_max_lateral_accel_mpss);
        pnh.getParam("max_longitudinal_accel_mpss", m_max_longitudinal_accel_mpss);
        pnh.getParam("max_yaw_rate_rps",            m_max_yaw_rate_rps);
        pnh.getParam("max_yaw_rate_rate_rpss",      m_max_yaw_rate_rate_rpss);
    }

    /// @brief Default destructor for forward declares
    LocalPlannerConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    const std::string&              getGoalTopic()                 const noexcept {return m_goal_topic;}
    const std::string&              getLocalPoseTopic()            const noexcept {return m_local_pose_topic;}
    float64_t                       getUpdateRateHz()              const noexcept {return m_update_rate_hz;}
    float64_t                       getTimeStepMs()                const noexcept {return m_time_step_ms;}
    float64_t                       getVelocityDiscretizationMps() const noexcept {return m_velocity_discretization_mps;}
    float64_t                       getYawRateDiscretizationRps()  const noexcept {return m_yaw_rate_discretization_mps;}
    float64_t                       getMaxVelMps()                 const noexcept {return m_max_vel_mps;}
    float64_t                       getMaxLateralAccelMpss()       const noexcept {return m_max_lateral_accel_mpss;}
    float64_t                       getMaxLongitudinalAccelMpss()  const noexcept {return m_max_longitudinal_accel_mpss;}
    float64_t                       getMaxYawRateRps()             const noexcept {return m_max_yaw_rate_rps;}
    float64_t                       getMaxYawRateRateRpss()        const noexcept {return m_max_yaw_rate_rate_rpss;}
    const GraphNodeToleranceConfig& getGraphNodeToleranceConfig()  const noexcept {return m_node_cfg;}
    /// @}

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
    float64_t m_time_step_ms{50};                  ///< Time step of the planner in ms
    float64_t m_velocity_discretization_mps{0.1};  ///< Step size of velocity, in mps
    float64_t m_yaw_rate_discretization_mps{0.25}; ///< Step size of yaw rate, in mps
    /// @}

    /// @brief Non Holonomic Constraints
    /// @{
    float64_t m_max_vel_mps{10.0};                  ///< Max magnitude of velocity, in mps
    float64_t m_max_lateral_accel_mpss{1.0};        ///< Max lateral acceleration, in mpss
    float64_t m_max_longitudinal_accel_mpss{1.0};   ///< Max longitudinal acceleration, in mpss
    float64_t m_max_yaw_rate_rps{1.5};              ///< Max yaw rate, in rps
    float64_t m_max_yaw_rate_rate_rpss{0.1};        ///< Max yaw rate rate, in rpss
    /// @}


    GraphNodeToleranceConfig m_node_cfg; ///< Graph node tolerance config
};

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_CONFIG_HPP
