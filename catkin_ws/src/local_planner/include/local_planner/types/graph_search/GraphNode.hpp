#ifndef PLANNING_GRAPH_NODE_HPP
#define PLANNING_GRAPH_NODE_HPP

// Component
#include "GraphNodeToleranceConfig.hpp"
#include "Point.hpp"

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/functional/hash.hpp>

// Standard
#include <cstdint>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store information about a graph node in a graph search
class GraphNode
{
public:
    /// @brief Default constructor
    GraphNode()
    {
        m_id = id_generator;    
        id_generator++;
    };

    /// @brief Default destructor
    ~GraphNode() = default;

    /// @brief Copy constructor
    GraphNode(const GraphNode&) noexcept = default;

    /// @brief Copy operator
    GraphNode& operator=(const GraphNode&) noexcept = default;

    /// @brief Move constructor
    GraphNode(GraphNode&&) noexcept = default;

    /// @brief Move operator
    GraphNode& operator=(GraphNode&&) noexcept = default;

    /// @brief Less than comparator operator for sorting graph nodes
    /// @param rhs Righthand side of comparator
    /// @return `true` if lhs is < rhs
    bool operator<(const GraphNode& rhs) const noexcept
    {
        return ((this->m_cost > rhs.m_cost) == true);
    }

    /// @brief Equality operator for checking equality of two nodes (tolerance based)
    /// @param rhs Righthand side of comparator
    /// @return `true` if rhs is equal
    bool operator==(const GraphNode& rhs) const noexcept
    {
        return ((std::fabs(this->m_point_m.getX() - rhs.m_point_m.getX()) < tolerance_cfg.getToleranceM()) &&
                (std::fabs(this->m_point_m.getY() - rhs.m_point_m.getY()) < tolerance_cfg.getToleranceM()));                
    }     

    /// @brief Static member variables
    /// @{
    static std::uint64_t            id_generator;  ///< ID generator of the graph nodes
    static GraphNodeToleranceConfig tolerance_cfg; ///< Tolerance configuration for equality
    /// @}

    /// @brief Accessor
    /// @return Val
    /// @{
    std::uint64_t    getID()                               const noexcept {return m_id;}
    std::uint64_t    getParentID()                         const noexcept {return m_parent_id;}
    const Point&     getPointM()                           const noexcept {return m_point_m;}
    float64_t        getG()                                const noexcept {return m_g;}
    float64_t        getCost()                             const noexcept {return m_cost;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setParentID(const std::uint64_t val)                     noexcept {m_parent_id = val;}
    void setEstimatedPointM(const Point& val)                     noexcept {m_point_m = val;}
    void setEstimatedPointM(Point&& val)                          noexcept {m_point_m = val;}
    void setG(const float64_t val)                                noexcept {m_g = val;}
    void setCost(const float64_t val)                             noexcept {m_cost = val;}
    /// @}

private:
    std::uint64_t m_id{0U};                           ///< ID of the node
    std::uint64_t m_parent_id{0U};                    ///< ID of the parent node
    Point         m_point_m{};              ///< Estimated cartesian coordinates of node
    float64_t     m_g{0.0};                           ///< G of the node
    float64_t     m_cost{0.0};                        ///< Cost
};

} // namespace local_planner

namespace std
{

/// @brief Class specialization of hash for Graph Nodes
template <>
class hash<local_planner::GraphNode>
{
public:
    /// @brief Default operator for hashing graph nodes
    /// @param node The node to be hashed
    /// @return hash
    size_t operator()(const local_planner::GraphNode& node) const noexcept
    {
        const std::size_t x_hash        = hash<std::size_t>()(static_cast<std::size_t>(std::round(node.getPointM().getX() /
                                                                                                  local_planner::GraphNode::tolerance_cfg.getToleranceM())));
        const std::size_t y_hash        = hash<std::size_t>()(static_cast<std::size_t>(std::round(node.getPointM().getY() /
                                                                                                  local_planner::GraphNode::tolerance_cfg.getToleranceM())));
                

        std::size_t seed = 0U;
        boost::hash_combine(seed, x_hash);
        boost::hash_combine(seed, y_hash);
        
        return hash<std::size_t>()(seed);
    }
};

} // namespace std

#endif // PLANNING_GRAPH_NODE_HPP
