#ifndef VI_VEHICLE_INTERFACE_CONFIG_HPP
#define VI_VEHICLE_INTERFACE_CONFIG_HPP

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
    VehicleInterfaceConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("local_traj_topic",            m_traj_topic);
        pnh.getParam("local_pose_topic",            m_local_pose_topic);        
        pnh.getParam("command_topic",               m_cmd_topic);
        pnh.getParam("update_rate_hz",              m_update_rate_hz);
    }

    /// @brief Default destructor for forward declares
    VehicleInterfaceConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    const std::string&              getTrajectoryTopic()           const noexcept {return m_traj_topic;}
    const std::string&              getLocalPoseTopic()            const noexcept {return m_local_pose_topic;}    
    const std::string&              getCommandTopic()              const noexcept {return m_cmd_topic;}
    float64_t                       getUpdateRateHz()              const noexcept {return m_update_rate_hz;}

private:
    /// @brief I/O Topics
    /// @{
    std::string m_traj_topic{""};       ///< Trajectory publish topic
    std::string m_local_pose_topic{""}; ///< Local pose topic      
    std::string m_cmd_topic{""};        ///< Command topic
    /// @}

    /// @brief Performance
    /// @{
    float64_t m_update_rate_hz{10.0}; ///< Update rate in hz
    /// @}
};

} // namespace vi

#endif // VI_VEHICLE_INTERFACE_CONFIG_HPP