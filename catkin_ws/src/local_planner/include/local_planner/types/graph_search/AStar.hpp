#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

// Component
#include "Costmap.hpp"
#include "GraphNode.hpp"
#include "LocalPlannerConfig.hpp"
#include "LocalPlannerData.hpp"
#include "Point.hpp"

// Ros
#include <nav_msgs/Path.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/shared_ptr.hpp>

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

    /// @brief Accessor for planned path
    /// @return Planned path
    /// @{
    const std::vector<Point>& getPath()    const noexcept {return m_path;}
    nav_msgs::Path::ConstPtr  getRosPath() const noexcept {return boost::make_shared<nav_msgs::Path>(m_ros_path);}
    /// @}

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
    /// @return `true` if successful
    bool initializePlanner() noexcept;

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

    /// @brief Checks for collision
    /// @param point The point to check for collision (uses inflated params)
    /// @return probability of collision
    std::int64_t getProbabilityCollision(const Point& pt);

    /// @brief Reconstructs path from the nodes after the goal is found
    /// @return `true` if successful
    bool reconstructPath();

    /// @brief Graph search node-related members
    /// @{
    std::priority_queue<GraphNode, std::vector<GraphNode>> m_frontier;     ///< P-q of graph nodes to use
    std::unordered_set<GraphNode>                          m_open_nodes;   ///< Open nodes
    std::unordered_map<std::uint64_t, GraphNode>           m_nodes;        ///< All nodes by ID, used for reconstruction
    GraphNode                                              m_goal_node;    ///< Goal node
    Costmap                                                m_costmap;      ///< Costmap for collision checking
    /// @}

    /// @brief Inputs and outputs
    /// @{
    std::vector<Point>                   m_path;       ///< Holonomic path found
    nav_msgs::Path                       m_ros_path;   ///< Ros Path
    std::shared_ptr<LocalPlannerConfig>  m_cfg;        ///< Config for the local planner
    LocalPlannerData                     m_data;       ///< Inbound data over IPC
    /// @}
    
};

} // namespace local_planner

#endif // PLANNING_ASTAR_HPP
