#ifndef PLANNING_TRAJECTORY_SOLVER_HELPER_HPP
#define PLANNING_TRAJECTORY_SOLVER_HELPER_HPP

// Component
#include "LocalPlannerData.hpp"
#include "Point.hpp"
#include "TrajectoryConfig.hpp"

// Ros
#include <autonomy_msgs/Trajectory.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for adding control commands to a holonomic path
class TrajectorySolver
{
public:
    /// @brief Default constructor
    /// @param cfg Trajectory config
    TrajectorySolver(const TrajectoryConfig& cfg);

    /// @brief Default destructor
    ~TrajectorySolver();

    /// @brief Mutator for data over IPC
    /// @param val The juicy, juicy data
    void setLocalPlannerData(const LocalPlannerData& val) noexcept {m_data = val;}

    /// @brief Calculates a trajectory from a path
    /// @param path The holonomic path
    /// @param now_s Current time
    /// @return Trajectory
    autonomy_msgs::Trajectory::ConstPtr calculateTrajectory(const std::vector<Point>& path, const ros::Time& now_s);

private:
    /// @brief Calculates distance based trajectory
    /// @param path The holonomic path    
    void calculateDistanceBasedTrajectory(const std::vector<Point>& path);

    /// @brief Calculates distance based trajectory
    /// @param now_s Current time
    void calculateTimeBasedTrajectory(const ros::Time& now_s);

    /// @brief Ensures an angle is between 0 and 2 pi
    /// @param angle The angle to correct
    void correctAngle(float64_t& angle);


    autonomy_msgs::Trajectory m_dist_based_traj; ///< Distance based trajectory
    autonomy_msgs::Trajectory m_time_based_traj; ///< Time based trajectory

    TrajectoryConfig m_cfg;  ///< Planning config
    LocalPlannerData m_data; ///< The data to work with over IPC

};    

} // namespace local_planner

#endif // PLANNING_TRAJECTORY_SOLVER_HELPER_HPP