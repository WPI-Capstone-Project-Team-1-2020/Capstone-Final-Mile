#ifndef PLANNING_COSTMAP_HPP
#define PLANNING_COSTMAP_HPP

// Component
#include "Point.hpp"
#include "RosConversionHelper.hpp"

// Ros
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <Eigen/Dense>

// Standard
#include <cstdint>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store information about occupied cells
class Costmap
{
public:
    /// @brief Mutator for setting the occupancy grid
    /// @param grid The occupancy grid
    /// @param pose The local pose of the point
    inline void setOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid, const geometry_msgs::Pose& pose)
    {
        m_grid = grid;              
        m_local_pose = pose;
    }

    inline std::int64_t getProbabilityOccupied(const Point& pt)
    {
        const float64_t resolution  = m_grid->info.resolution;
        const float64_t width       = m_grid->info.width;
        const float64_t heading_r   = RosConversionHelper::quaternionMsgToYawR(m_local_pose.orientation);
        const float64_t base_x_m    =  (pt.getX() - m_local_pose.position.x)*std::cos(heading_r) + (pt.getY() - m_local_pose.position.y)*std::sin(heading_r);
        const float64_t base_y_m    = -(pt.getX() - m_local_pose.position.x)*std::sin(heading_r) + (pt.getY() - m_local_pose.position.y)*std::cos(heading_r);        

        const std::size_t x = static_cast<std::size_t>((base_x_m - m_grid->info.origin.position.x)/resolution);
        const std::size_t y = static_cast<std::size_t>((base_y_m - m_grid->info.origin.position.y)/resolution);

        const std::size_t grid_it = x + y*static_cast<std::size_t>(width); 

        if (grid_it > m_grid->data.size())
        {
            return 0;
        }

        return static_cast<std::int64_t>(m_grid->data[grid_it]);
    }

private:
    nav_msgs::OccupancyGrid::ConstPtr       m_grid;        ///< Occupancy Grid
    geometry_msgs::Pose                     m_local_pose;  ///< Local Pose
};

} // namespace local_planner

#endif // PLANNING_COSTMAP_HPP
