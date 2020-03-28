#ifndef VI_VEHICLE_INTERFACE_CONFIG_HPP
#define VI_VEHICLE_INTERFACE_CONFIG_HPP

// Component
#include "PIDConfig.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

// Ros
#include <ros/ros.h>

// Standard
#include <cstdint>
#include <string>

namespace vi
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for vehicle interface
class VehicleInterfaceConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle used to snipe params
    VehicleInterfaceConfig(ros::NodeHandle& pnh) :
        m_linear_config{pnh, "linear"},
        m_angular_config{pnh, "angular"}
    {
        pnh.getParam("goal_reached_topic",          m_goal_reached_topic);
        pnh.getParam("trajectory_topic",            m_traj_topic);
        pnh.getParam("local_pose_topic",            m_local_pose_topic);        
        pnh.getParam("command_topic",               m_cmd_topic);
        pnh.getParam("update_rate_hz",              m_update_rate_hz);
    }

    /// @brief Default destructor for forward declares
    VehicleInterfaceConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    const std::string& getGoalReachedTopic() const noexcept {return m_goal_reached_topic;}
    const std::string& getTrajectoryTopic()  const noexcept {return m_traj_topic;}
    const std::string& getLocalPoseTopic()   const noexcept {return m_local_pose_topic;}    
    const std::string& getCommandTopic()     const noexcept {return m_cmd_topic;}
    float64_t          getUpdateRateHz()     const noexcept {return m_update_rate_hz;}
    const PIDConfig&   getLinearPIDConfig()  const noexcept {return m_linear_config;}
    const PIDConfig&   getAngularPIDConfig() const noexcept {return m_angular_config;}

private:
    /// @brief I/O Topics
    /// @{
    std::string m_goal_reached_topic{""}; ///< Goal reached topic
    std::string m_traj_topic{""};         ///< Trajectory publish topic
    std::string m_local_pose_topic{""};   ///< Local pose topic      
    std::string m_cmd_topic{""};          ///< Command topic
    /// @}

    /// @brief Performance
    /// @{
    float64_t m_update_rate_hz{10.0}; ///< Update rate in hz
    /// @}

    /// @brief PID Configs
    /// @{
    PIDConfig m_linear_config;  ///< Linear velocity config
    PIDConfig m_angular_config; ///< Angular velocity config
    /// @}
};

} // namespace vi

#endif // VI_VEHICLE_INTERFACE_CONFIG_HPP
