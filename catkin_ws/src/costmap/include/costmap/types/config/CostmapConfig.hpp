#ifndef CM_VEHICLE_INTERFACE_CONFIG_HPP
#define CM_VEHICLE_INTERFACE_CONFIG_HPP

// Libraries
#include <boost/cstdfloat.hpp>

// Ros
#include <ros/ros.h>

// Standard
#include <string>

namespace cm
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for costmap
class CostmapConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle used to snipe params
    CostmapConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("pointcloud_topic",     m_pc_topic);
        pnh.getParam("costmap_topic",        m_cm_topic);
        pnh.getParam("local_pose_topic",     m_local_pose_topic);
        pnh.getParam("diagnostics_topic",    m_diagnostics_topic);
        pnh.getParam("update_rate_hz",       m_update_rate_hz);
        pnh.getParam("height_m",             m_height_m);
        pnh.getParam("width_m",              m_width_m);
        pnh.getParam("resolution_m",         m_resolution_m);        
        pnh.getParam("inflation_m",          m_inflation_m);
    }

    /// @brief Default destructor for forward declares
    CostmapConfig() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    const std::string& getPointCloudTopic()    const noexcept {return m_pc_topic;}
    const std::string& getCostmapTopic()       const noexcept {return m_cm_topic;}
    const std::string& getLocalPoseTopic()     const noexcept {return m_local_pose_topic;}
    const std::string& getDiagnosticsTopic()   const noexcept {return m_diagnostics_topic;}
    float64_t          getUpdateRateHz()       const noexcept {return m_update_rate_hz;}
    float64_t          getCostmapHeightM()     const noexcept {return m_height_m;}
    float64_t          getCostmapWidthM()      const noexcept {return m_width_m;}
    float64_t          getCostmapResolutionM() const noexcept {return m_resolution_m;}
    float64_t          getCostmapInflationM()  const noexcept {return m_inflation_m;}
    /// @}

private:
    /// @brief I/O Topics
    /// @{
    std::string m_pc_topic{""};              ///< Pointcloud topic
    std::string m_cm_topic{""};              ///< Costmap topic
    std::string m_local_pose_topic{""};      ///< Local pose topic      
    std::string m_diagnostics_topic{""};     ///< Diagnostics topic
    /// @}

    float64_t m_update_rate_hz{10.0}; ///< Update rate

    /// @brief Map parameters
    /// @{
    float64_t m_height_m{60.0};    ///< Height of cm, in meters
    float64_t m_width_m{60.0};     ///< Width of cm, in meters
    float64_t m_resolution_m{0.1}; ///< Resolution of cm, in meters
    float64_t m_inflation_m{10.0}; ///< Inflation radius, in meters
    /// @}
};

} // namespace CM

#endif // CM_VEHICLE_INTERFACE_CONFIG_HPP
