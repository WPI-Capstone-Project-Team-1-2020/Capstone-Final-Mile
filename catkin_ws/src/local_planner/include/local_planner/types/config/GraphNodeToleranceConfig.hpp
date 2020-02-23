#ifndef PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP
#define PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold equality checking tolerance parameters
class GraphNodeToleranceConfig
{
public:
    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t getXToleranceM()         const noexcept {return m_x_tol_m;}
    float64_t getYToleranceM()         const noexcept {return m_y_tol_m;}
    float64_t getSpeedToleranceMps()   const noexcept {return m_speed_tol_mps;}
    float64_t getHeadingToleranceRad() const noexcept {return m_heading_tol_rad;}
    float64_t getYawRateToleranceRps() const noexcept {return m_yaw_rate_tol_rps;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setXToleranceM(const float64_t val)         noexcept {m_x_tol_m = val;}
    void setYToleranceM(const float64_t val)         noexcept {m_y_tol_m = val;}
    void setSpeedToleranceMps(const float64_t val)   noexcept {m_speed_tol_mps = val;}
    void setHeadingToleranceRad(const float64_t val) noexcept {m_heading_tol_rad = val;}
    void setYawRateToleranceRps(const float64_t val) noexcept {m_yaw_rate_tol_rps = val;}
    /// @}  

private:
    float64_t m_x_tol_m{0.0};          ///< Tolerance on x in meters
    float64_t m_y_tol_m{0.0};          ///< Tolerance on y in meters
    float64_t m_speed_tol_mps{0.0};    ///< Tolerance on speed in meters per second
    float64_t m_heading_tol_rad{0.0};  ///< Tolerance on heading in radians
    float64_t m_yaw_rate_tol_rps{0.0}; ///< Tolerance on yaw rate in rps    
}; 

} // namespace local_planner

#endif // PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP