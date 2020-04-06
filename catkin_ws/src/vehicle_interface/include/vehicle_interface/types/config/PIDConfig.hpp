#ifndef VI_PID_CONFIG_HPP
#define VI_PID_CONFIG_HPP

// Libraries
#include <boost/cstdfloat.hpp>

// Ros
#include <ros/ros.h>

// Standard
#include <cstdint>
#include <string>

namespace vi
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for vehicle interface
class PIDConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle used to snipe params
    PIDConfig(ros::NodeHandle& pnh, const std::string& prefix)
    {
        pnh.getParam(prefix + "_p",     m_p);
        pnh.getParam(prefix + "_i",     m_i);
        pnh.getParam(prefix + "_d",     m_d);        
        pnh.getParam("max_dt_s",        m_max_dt_s);
        pnh.getParam("ctl_buffer_size", m_ctl_buffer_size);
    }

    /// @brief Default destructor for forward declares
    ~PIDConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    float64_t    getPGain()             const noexcept {return m_p;}
    float64_t    getIGain()             const noexcept {return m_i;}
    float64_t    getDGain()             const noexcept {return m_d;}
    float64_t    getMaxDtS()            const noexcept {return m_max_dt_s;}
    std::int32_t getControlBufferSize() const noexcept {return m_ctl_buffer_size;}
    /// @}

private:
    /// @brief Gains
    /// @{
    float64_t m_p{0.0}; ///< Proportional gain
    float64_t m_i{0.0}; ///< Integral gain
    float64_t m_d{0.0}; ///< Derivative gain
    /// @}

    /// @brief Sanity checks
    /// @{
    float64_t m_max_dt_s; ///< Max allowable DT in seconds
    /// @}

    /// @brief Filter configs
    /// @{
    std::int32_t m_ctl_buffer_size{100};
    /// @}
};

} // namespace vi

#endif // VI_PID_CONFIG_HPP
