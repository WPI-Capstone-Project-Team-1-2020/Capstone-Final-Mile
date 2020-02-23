#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

// Component
#include "GraphNode.hpp"
#include "LocalPlannerConfig.hpp"
#include "LocalPlannerData.hpp"
#include "Point.hpp"

// Libraries
#include <boost/cstdfloat.hpp>
#include <unsupported/Eigen/Splines>

// Ros
#include <autonomy_msgs/Trajectory.h>

// Standard
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace local_planner
{

using float64_t = boost::float64_t;                     ///< Alias for 64 bit float
using Spline1d = Eigen::Spline<float64_t, 2U>;           ///< Alias for 1-d spline
using Spline1dFitting = Eigen::SplineFitting<Spline1d>; ///< Alias for spline fitting

/// @brief Class to do graph search things
class AStar
{
public:
    /// @brief Default constructor
    AStar(const std::shared_ptr<LocalPlannerConfig>& cfg);

    /// @brief Default destructor
    ~AStar();

    /// @brief Main driving function of the solver
    /// @return `true` if udpate was successful and trajectory was found
    bool update();

    /// @brief Accessor for planned trajectory
    /// @return Planned trajectory
    const autonomy_msgs::Trajectory& getTrajectory() const noexcept {return m_trajectory;}

    /// @brief Mutator for the local planner data
    /// @param val The data
    void setLocalPlannerData(const LocalPlannerData& data) noexcept {m_data = data;}

private:
    /// @brief Resets the planner
    void resetPlanner() noexcept;

    /// @brief Plans a trajectory
    void planTrajectory();



    /// @brief Graph search node-related members
    /// @{
    std::priority_queue<GraphNode, std::vector<GraphNode>> m_frontier;     ///< P-q of graph nodes to use
    std::unordered_set<GraphNode>                          m_open_nodes;   ///< Open nodes
    std::unordered_set<GraphNode>                          m_closed_nodes; ///< Closed nodes    
    /// @}

    /// @brief Inputs and outputs
    /// @{
    autonomy_msgs::Trajectory            m_trajectory; ///< Trajectory to send out to controller
    std::shared_ptr<LocalPlannerConfig>  m_cfg;        ///< Config for the local planner
    LocalPlannerData                     m_data;       ///< Inbound data over IPC
    /// @}
    
};

} // namespace local_planner

#endif // PLANNING_ASTAR_HPP
