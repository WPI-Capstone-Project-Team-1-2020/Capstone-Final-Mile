// Libraries
#include <boost/cstdfloat.hpp>

// Standard
#include <cstdint>

namespace planning
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store information about a graph node in a graph search
class GraphNode
{
public:
    /// @brief Default constructor
    GraphNode(){id++;}

    /// @brief Default destructor
    ~GraphNode() = default;

    /// @brief Default operator for sorting graph nodes
    bool operator()(const GraphNode& lhs, const GraphNode& rhs) const noexcept
    {
        return lhs.m_cost > rhs.m_cost;
    }

    static std::uint64_t id; ///< ID of the graph node

    /// @brief Accessor
    /// @return Val
    /// @{
    std::uint64_t    getParentID()             const noexcept {return m_parent_id;}
    float64_t        getEstimatedHeadingR()    const noexcept {return m_estimated_heading_r;}
    float64_t        getEstimatedYawRateRps()  const noexcept {return m_estimated_yaw_rate_rps;}
    float64_t        getEstimatedVelocityMps() const noexcept {return m_estimated_velocity_mps;}
    float64_t        getCommandedYawRateRps()  const noexcept {return m_commanded_yaw_rate_rps;}
    float64_t        getCommandedVelocityMps() const noexcept {return m_commanded_velocity_mps;}
    float64_t        getG()                    const noexcept {return m_g;}
    float64_t        getCost()                 const noexcept {return m_cost;}
    /// @}

    /// @brief Mutator
    /// @param Val val
    /// @{
    void setParentID(const std::uint64_t val)         noexcept {m_parent_id = val;}
    void setEstimatedHeadingR(const float64_t val)    noexcept {m_estimated_heading_r = val;}
    void setEstimatedYawRateRps(const float64_t val)  noexcept {m_estimated_yaw_rate_rps = val;}
    void setEstimatedVelocityMps(const float64_t val) noexcept {m_estimated_velocity_mps = val;}
    void setCommandedYawRateRps(const float64_t val)  noexcept {m_commanded_yaw_rate_rps = val;}
    void setCommandedVelocityMps(const float64_t val) noexcept {m_commanded_velocity_mps = val;}
    /// @}

private:

    std::uint64_t m_parent_id{0U};                ///< ID of the parent node
    float64_t     m_estimated_heading_r{0.0};     ///< Estimated Heading in radians
    float64_t     m_estimated_yaw_rate_rps{0.0};  ///< Estimated Yaw rate in radians per second
    float64_t     m_estimated_velocity_mps{0.0};  ///< Estimated Velocity in meters per second
    float64_t     m_commanded_yaw_rate_rps{0.0};  ///< Commanded Yaw rate in radians per second
    float64_t     m_commanded_velocity_mps{0.0};  ///< Commanded Velocity in meters per second
    float64_t     m_g{0.0};                       ///< G of the node
    float64_t     m_cost{0.0};                    ///< Cost
};

}