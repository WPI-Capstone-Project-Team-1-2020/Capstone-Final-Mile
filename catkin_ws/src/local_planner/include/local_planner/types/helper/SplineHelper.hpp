#ifndef PLANNING_SPLINE_HELPER_HPP
#define PLANNING_SPLINE_HELPER_HPP

// Component
#include "GraphNode.hpp"

// Libraries
#include <boost/cstdfloat.hpp>
#include <unsupported/Eigen/Splines>


namespace local_planner
{

using float64_t = boost::float64_t;                     ///< Alias for 64 bit float
using Spline1d = Eigen::Spline<float64_t, 2U>;          ///< Alias for 1-d spline
using Spline1dFitting = Eigen::SplineFitting<Spline1d>; ///< Alias for spline fitting    

/// @brief Class for splining
class SplineHelper
{
public:
    /// @brief Calculates target heading for heuristic calculations
    /// @param current_node The current node
    /// @param goal_node The goal node
    /// @param spline_order Order of the spline
    /// @param time_step_ms Time step of the local planner
    /// @return Target heading in radians
    static float64_t calcTargetHeadingR(const GraphNode& current_node, const GraphNode& goal_node, const float64_t spline_order, const float64_t time_step_ms);

    /// @brief Calculates a spline trajectory from current node to goal node, used to calculate target heading
    /// @param current_node The current node
    /// @param goal_node The goal node
    /// @param spline_order Order of the spline
    /// @return Target spline
    Spline1d calcTargetSpline(const GraphNode& current_node, const GraphNode& goal_node, const float64_t spline_order);
};

} // namespace local_planner

#endif // PLANNING_SPLINE_HELPER_HPP