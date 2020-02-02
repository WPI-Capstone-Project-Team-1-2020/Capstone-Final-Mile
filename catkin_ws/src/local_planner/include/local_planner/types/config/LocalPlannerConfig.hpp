#ifndef PLANNING_LOCAL_PLANNER_CONFIG_HPP
#define PLANNING_LOCAL_PLANNER_CONFIG_HPP

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

    }

    /// @brief Default destructor for forward declares
    LocalPlannerConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{

    /// @}

    /// @brief Mutator
    /// @param Val val
    /// @{

    /// @}

private:
    std::string m_goal_topic;
    std::string m_local_pose_topic;
    std::string m_veh_state_topic;
    std::string m_command_topic;    
};

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_CONFIG_HPP
