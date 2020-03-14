#ifndef PLANNING_NODE_DENSITY_CONFIG_HPP
#define PLANNING_NODE_DENSITY_CONFIG_HPP

// Ros
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold node density related configs
class NodeDensityConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle to snipe those params
    NodeDensityConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("bin_size_m",        m_bin_size_m);
        pnh.getParam("num_nodes_per_bin", m_num_nodes_per_bin);
        pnh.getParam("grid_size_m",       m_grid_size_m);
    }

    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t   getBinSizeM()          const noexcept {return m_bin_size_m;}
    std::size_t getNumberNodesPerBin() const noexcept {return m_num_nodes_per_bin;}
    float64_t   getGridSizeM()         const noexcept {return m_grid_size_m;}
    /// @}

private:
    float64_t    m_bin_size_m{1.0};         ///< Size of the square bins in meters
    std::int32_t m_num_nodes_per_bin{100U}; ///< Max number of nodes per bin
    float64_t    m_grid_size_m{50.0};       ///< Size of one side of the square grid in meters
}; 

} // namespace local_planner

#endif // PLANNING_NODE_DENSITY_CONFIG_HPP