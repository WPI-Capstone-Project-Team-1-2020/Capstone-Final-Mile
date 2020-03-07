#ifndef PLANNING_FORWARD_SIM_HELPER_HPP
#define PLANNING_FORWARD_SIM_HELPER_HPP

// Component
#include "GraphNode.hpp"

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

    /// @brief Forward simulates a graph node over a horizon of a time step
    /// @param node The node to forward sim
    /// @param parent_node The parent node, used to calc accels
    /// @param time_step_ms The horizon in ms
    static void forwardSimGraphNode(GraphNode& node, const GraphNode& parent_node, const float64_t time_step_ms);
};    

} // namespace local_planner

#endif // PLANNING_FORWARD_SIM_HELPER_HPP