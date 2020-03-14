#ifndef PLANNING_FORWARD_SIM_HELPER_HPP
#define PLANNING_FORWARD_SIM_HELPER_HPP

// Ros
#include <nav_msgs/Odometry.h>

namespace local_planner
{

/// @brief Class for integrating callback data to a given time
class ForwardSimHelper
{
public:
    /// @brief Forward simulates a pose to the given time
    /// @param pose Current local pose
    /// @param now_s Time to integrate to in seconds
    /// @return Integrated local pose
    static nav_msgs::Odometry::ConstPtr forwardSimPose(const nav_msgs::Odometry::ConstPtr& pose, const ros::Time& now_s);    
};    

} // namespace local_planner

#endif // PLANNING_FORWARD_SIM_HELPER_HPP