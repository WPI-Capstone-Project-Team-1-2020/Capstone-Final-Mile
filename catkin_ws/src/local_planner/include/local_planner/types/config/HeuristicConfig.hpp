#ifndef PLANNING_HEURISTIC_CONFIG_HPP
#define PLANNING_HEURISTIC_CONFIG_HPP

// Ros
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to hold equality checking tolerance parameters
class HeuristicConfig
{
public:
    /// @brief Default constructor
    /// @param pnh Private nodehandle to snipe those params
    HeuristicConfig(ros::NodeHandle& pnh)
    {
        pnh.getParam("dist_weight",    m_dist_weight);
        pnh.getParam("heading_weight", m_heading_weight);
        pnh.getParam("speed_weight",   m_speed_weight);
    }

    /// @brief Accesor
    /// @return Val
    /// @{
    float64_t getDistWeight()    const noexcept {return m_dist_weight;}
    float64_t getHeadingWeight() const noexcept {return m_heading_weight;}
    float64_t getSpeedWeight()   const noexcept {return m_speed_weight;}
    /// @}

private:
    float64_t m_dist_weight{100.0};     ///< Weight for the heuristic of distance
    float64_t m_heading_weight{100.0};  ///< Weight for the heuristic of heading
    float64_t m_speed_weight{100.0};    ///< Weight for the heuristic of speed
}; 

} // namespace local_planner

#endif // PLANNING_HEURISTIC_CONFIG_HPP