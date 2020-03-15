#ifndef PLANNING_TRAJECTORY_SOLVER_HELPER_HPP
#define PLANNING_TRAJECTORY_SOLVER_HELPER_HPP

// Component
#include "LocalPlannerConfig.hpp"
#include "LocalPlannerData.hpp"
#include "Point.hpp"

// Ros
#include <autonomy_msgs/Trajectory.h>

// Libraries
#include <boost/cstdfloat.hpp>

// Standard
#include <memory>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for adding control commands to a holonomic path
class TrajectorySolver
{
public:
    /// @brief Default constructor
    TrajectorySolver(const std::shared_ptr<LocalPlannerConfig>& cfg);

    /// @brief Default destructor
    ~TrajectorySolver();

    /// @brief Mutator for data over IPC
    /// @param val The juicy, juicy data
    void setLocalPlannerData(const LocalPlannerData& val) noexcept {m_data = val;}

    /// @brief Calculates a trajectory from a path
    /// @param path The holonomic path
    /// @return Trajectory
    autonomy_msgs::Trajectory::ConstPtr calculateTrajectory(const std::vector<Point>& path);

private:
    std::shared_ptr<LocalPlannerConfig> m_cfg;  ///< Planning config
    LocalPlannerData                    m_data; ///< The data to work with over IPC

};    

} // namespace local_planner

#endif // PLANNING_TRAJECTORY_SOLVER_HELPER_HPP