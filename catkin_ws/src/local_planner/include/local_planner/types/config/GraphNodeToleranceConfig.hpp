#ifndef PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP
#define PLANNING_GRAPH_NODE_TOLERANCE_CONFIG_HPP

// Ros
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold equality checking tolerance parameters
class GraphNodeToleranceConfig
{
public:
    /// @brief Default constructor
    GraphNodeToleranceConfig() = default;

    /// @brief Constructor that actually gets the parameters
    /// @param pnh Private nodehandle to snipe those params
    GraphNodeToleranceConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("x_tolerance_m",          m_x_tol_m);
        pnh.getParam("y_tolerance_m",          m_y_tol_m);
        pnh.getParam("speed_tolerance_m",      m_speed_tol_mps);
        pnh.getParam("heading_tolerance_r",    m_heading_tol_rad);
        pnh.getParam("yaw_rate_tolerance_rps", m_yaw_rate_tol_rps);
    }

    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t getXToleranceM()         const noexcept {return m_x_tol_m;}
    float64_t getYToleranceM()         const noexcept {return m_y_tol_m;}
    float64_t getSpeedToleranceMps()   const noexcept {return m_speed_tol_mps;}
    float64_t getHeadingToleranceR()   const noexcept {return m_heading_tol_rad;}
    float64_t getYawRateToleranceRps() const noexcept {return m_yaw_rate_tol_rps;}
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