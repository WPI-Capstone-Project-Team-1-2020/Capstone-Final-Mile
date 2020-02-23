#ifndef PLANNING_GRAPH_NODE_HPP
#define PLANNING_GRAPH_NODE_HPP

// Component
#include "GraphNodeToleranceConfig.hpp"
#include "Point.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

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
        id++;
        m_id = id;
    }

    /// @brief Default destructor
    ~GraphNode() = default;

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
        return ((std::fabs(this->m_estimated_point_m.getX() - rhs.m_estimated_point_m.getX()) < tolerance_cfg.getXToleranceM()) &&
                (std::fabs(this->m_estimated_point_m.getY() - rhs.m_estimated_point_m.getY()) < tolerance_cfg.getYToleranceM()) &&
                (std::fabs(this->m_estimated_velocity_mps   - rhs.m_estimated_velocity_mps)   < tolerance_cfg.getSpeedToleranceMps()) &&
                (std::fabs(this->m_estimated_heading_r      - rhs.m_estimated_heading_r)      < tolerance_cfg.getHeadingToleranceRad()) &&
                (std::fabs(this->m_estimated_yaw_rate_rps   - rhs.m_estimated_yaw_rate_rps)   < tolerance_cfg.getYawRateToleranceRps()));
    }     

    /// @brief Static member variables
    /// @{
    static std::uint64_t id;                       ///< ID count of the graph nodes
    static GraphNodeToleranceConfig tolerance_cfg; ///< Tolerance configuration for equality
    /// @}

    /// @brief Accessor
    /// @return Val
    /// @{
    std::uint64_t    getID()                   const noexcept {return m_id;}
    std::uint64_t    getParentID()             const noexcept {return m_parent_id;}
    const Point&     getEstimatedPointM()      const noexcept {return m_estimated_point_m;}
    float64_t        getEstimatedHeadingR()    const noexcept {return m_estimated_heading_r;}
    float64_t        getEstimatedYawRateRps()  const noexcept {return m_estimated_yaw_rate_rps;}
    float64_t        getEstimatedVelocityMps() const noexcept {return m_estimated_velocity_mps;}
    float64_t        getCommandedYawRateRps()  const noexcept {return m_commanded_yaw_rate_rps;}
    float64_t        getCommandedVelocityMps() const noexcept {return m_commanded_velocity_mps;}
    float64_t        getG()                    const noexcept {return m_g;}
    float64_t        getCost()                 const noexcept {return m_cost;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setParentID(const std::uint64_t val)         noexcept {m_parent_id = val;}
    void setEstimatedPointM(const Point& val)         noexcept {m_estimated_point_m = val;}
    void setEstimatedPointM(Point&& val)              noexcept {m_estimated_point_m = val;}
    void setEstimatedHeadingR(const float64_t val)    noexcept {m_estimated_heading_r = val;}
    void setEstimatedYawRateRps(const float64_t val)  noexcept {m_estimated_yaw_rate_rps = val;}
    void setEstimatedVelocityMps(const float64_t val) noexcept {m_estimated_velocity_mps = val;}
    void setCommandedYawRateRps(const float64_t val)  noexcept {m_commanded_yaw_rate_rps = val;}
    void setCommandedVelocityMps(const float64_t val) noexcept {m_commanded_velocity_mps = val;}
    /// @}

private:
    std::uint64_t m_id{0U};                       ///< ID of the node
    std::uint64_t m_parent_id{0U};                ///< ID of the parent node
    Point         m_estimated_point_m{};          ///< Estimated cartesian coordinates of node
    float64_t     m_estimated_heading_r{0.0};     ///< Estimated Heading in radians
    float64_t     m_estimated_yaw_rate_rps{0.0};  ///< Estimated Yaw rate in radians per second
    float64_t     m_estimated_velocity_mps{0.0};  ///< Estimated Velocity in meters per second
    float64_t     m_commanded_yaw_rate_rps{0.0};  ///< Commanded Yaw rate in radians per second
    float64_t     m_commanded_velocity_mps{0.0};  ///< Commanded Velocity in meters per second
    float64_t     m_g{0.0};                       ///< G of the node
    float64_t     m_cost{0.0};                    ///< Cost
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
        return hash<std::uint64_t>()(node.getID());
    }
};

} // namespace std

#endif // PLANNING_GRAPH_NODE_HPP
