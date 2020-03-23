#ifndef PLANNING_GOAL_CHECKER_HELPER_HPP
#define PLANNING_GOAL_CHECKER_HELPER_HPP

// Component
#include "LocalPlannerData.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for integrating callback data to a given time
class GoalChecker
{
public:
    /// @brief Determines whether goal has been reached
    /// @param data Local Planner data
    /// @param tol_m Tolerance, in meters
    /// @return `true` if goal has been reached
    static bool checkGoalReached(const LocalPlannerData& data, const float64_t tol_m) noexcept;
};    

} // namespace local_planner

#endif // PLANNING_GOAL_CHECKER_HELPER_HPP
