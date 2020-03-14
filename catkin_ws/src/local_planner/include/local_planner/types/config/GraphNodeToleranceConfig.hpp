#ifndef PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP
#define PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP

// Ros
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold equality checking tolerance parameters
class GraphNodeToleranceConfig
{
public:
    /// @brief Default constructor
    GraphNodeToleranceConfig() = default;

    /// @brief Constructor that actually gets the parameters
    /// @param pnh Private nodehandle to snipe those params
    GraphNodeToleranceConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("graph_discretization_m",          m_tol_m);
    }

    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t getToleranceM()         const noexcept {return m_tol_m;}    
    /// @}

private:
    float64_t m_tol_m{0.0};          ///< Tolerance on x in meters
}; 

} // namespace local_planner

#endif // PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP