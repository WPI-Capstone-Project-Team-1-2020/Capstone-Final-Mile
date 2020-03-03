#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

// Component
#include "GraphNode.hpp"
#include "LocalPlannerConfig.hpp"
#include "LocalPlannerData.hpp"
#include "Point.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

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

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to do graph search things
class AStar
{
public:
    /// @brief Default constructor
    /// @param cfg Local planner config
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
    /// @{
    void setLocalPlannerData(const LocalPlannerData& data) noexcept {m_data = data;}
    void setLocalPlannerData(LocalPlannerData&& data)      noexcept {m_data = data;}
    /// @}

private:
    /// @brief Resets the planner
    void resetPlanner() noexcept;

    /// @brief Initializes the planner
    void initializePlanner() noexcept;

    /// @brief Plans a trajectory
    void planTrajectory();

    /// @brief Expands the frontier
    /// @param current_node The node to expand from
    void expandFrontier(const GraphNode& current_node);

    /// @brief Gets neighbors of the current node
    /// @param current_node The node to find neighbors of
    std::vector<GraphNode> calcNeighbors(const GraphNode& current_node);

    /// @brief Calculates possible longitudinal velocities for neighboring nodes
    /// @param current_node The node to find possible velocities for
    /// @return vector of possible velocities
    std::vector<float64_t> calcPossibleLongitudinalVelocitiesMps(const GraphNode& current_node) const noexcept;

    /// @brief Calculates possible lateral velocities for neighboring nodes
    /// @param current_node The node to find possible velocities for
    /// @return vector of possible velocities
    std::vector<float64_t> calcPossibleLateralVelocitiesMps(const GraphNode& current_node) const noexcept;

    /// @brief Calculates possible yaw rates for neighboring nodes
    /// @param current_node The node to find possible yaw rates for
    /// @return vector of possible yaw rates
    std::vector<float64_t> calcPossibleYawRatesRps(const GraphNode& current_node) const noexcept;

    /// @brief Calculates G-score of a node
    /// @param parent_node The parent node of the new node
    /// @param node The node to calculate score of
    /// @return G-score
    float64_t calcNodeGScore(const GraphNode& parent_node, const GraphNode& node) const noexcept;

    /// @brief Calculates the cost of a node
    /// @param node The node to calculate score of
    /// @param target_heading_r The target heading, in radians
    /// @return The cost of the node
    float64_t calcNodeCost(const GraphNode& node, const float64_t target_heading_r) const;

    /// @brief Calculates the heuristic of a node
    /// @param node The node to get the heuristic of
    /// @param target_heading_r The target heading, in radians
    /// @return Heuristic of the node
    float64_t calcHeuristic(const GraphNode& node, const float64_t target_heading_r) const;

    /// @brief Determines if the vehicle needs to start slowing down to meet its goal
    /// @param current_node The current node of the graph search: this is not the new one being evaluated
    /// @param heading_r Heading of the new node in radians
    /// @param velocity_mps Current velocity of the node
    /// @return `true` if vehicle needs to start slowing down
    bool nodeNeedsToSlow(const GraphNode& current_node, const float64_t heading_r, const float64_t velocity_mps) const;

    /// @brief Graph search node-related members
    /// @{
    std::priority_queue<GraphNode, std::vector<GraphNode>> m_frontier;     ///< P-q of graph nodes to use
    std::unordered_set<GraphNode>                          m_open_nodes;   ///< Open nodes
    std::unordered_set<GraphNode>                          m_closed_nodes; ///< Closed nodes    
    GraphNode                                              m_goal_node;    ///< Goal node
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
