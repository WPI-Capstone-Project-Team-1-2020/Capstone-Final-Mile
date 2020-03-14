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
    /// @return `true` if successful
    bool planTrajectory();

    /// @brief Expands the frontier
    /// @param current_node The node to expand from
    void expandFrontier(const GraphNode& current_node);

    /// @brief Gets neighbors of the current node
    /// @param current_node The node to find neighbors of
    std::vector<GraphNode> calcNeighbors(const GraphNode& current_node);

    /// @brief Calculates G-score of a node
    /// @param parent_node The parent node of the new node
    /// @param node The node to calculate score of
    /// @return G-score
    float64_t calcNodeGScore(const GraphNode& parent_node, const GraphNode& node) const noexcept;

    /// @brief Calculates the cost of a node
    /// @param node The node to calculate score of    
    /// @return The cost of the node
    float64_t calcNodeCost(const GraphNode& node) const;

    /// @brief Graph search node-related members
    /// @{
    std::priority_queue<GraphNode, std::vector<GraphNode>> m_frontier;     ///< P-q of graph nodes to use
    std::unordered_set<GraphNode>                          m_open_nodes;   ///< Open nodes
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
