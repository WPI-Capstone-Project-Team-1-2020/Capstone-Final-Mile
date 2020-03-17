#ifndef PLANNING_TRAJECTORY_CONFIG_HPP
#define PLANNING_TRAJECTORY_CONFIG_HPP

// Ros
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold equality checking tolerance parameters
class TrajectoryConfig
{
public:
    /// @brief Default constructor
    TrajectoryConfig() = default;

    /// @brief Constructor that actually gets the parameters
    /// @param pnh Private nodehandle to snipe those params
    TrajectoryConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("time_step_ms",          m_time_step_ms);
        pnh.getParam("max_speed_mps",         m_max_speed_mps);
        pnh.getParam("max_accel_mps2",        m_max_accel_mps2);
        pnh.getParam("max_yaw_rate_rps",      m_max_yaw_rate_rps);
    }

    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t getTimeStepMs()    const noexcept {return m_time_step_ms;}    
    float64_t getMaxSpeedMps()   const noexcept {return m_max_speed_mps;}
    float64_t getMaxAccelMps2()  const noexcept {return m_max_accel_mps2;}
    float64_t getMaxYawRateRps() const noexcept {return m_max_yaw_rate_rps;}
    /// @}

private:
    float64_t m_time_step_ms{100.0};          ///< Time step of the trajectory, in ms
    float64_t m_max_speed_mps{10.0};          ///< Max speed of the vehicle, mps
    float64_t m_max_accel_mps2{1.0};          ///< Max accel, mps2
    float64_t m_max_yaw_rate_rps{1.0};        ///< Max yaw rate, rps
}; 

} // namespace local_planner

#endif // PLANNING_TRAJECTORY_CONFIG_HPP