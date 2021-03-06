#ifndef CM_GRID_BUILDER_HPP
#define CM_GRID_BUILDER_HPP

// Component
#include "CostmapData.hpp"

// Ros
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>

// Libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Standard
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace cm
{

using InflationCluster = std::vector<std::pair<std::size_t, std::int8_t>>;  ///< Alias for a set of points and corresponding probabilities
using InflationTable   = std::unordered_map<std::size_t, InflationCluster>; ///< Alias for this gross type I'm not defining again

// Forward declares
class CostmapConfig;    

/// @brief Class for building an occupancy grid from a pointcloud
class GridBuilder
{
public:
    /// @brief Default constructor    
    /// @param cfg config for the costmap
    GridBuilder(std::shared_ptr<CostmapConfig> cfg);

    /// @brief Default destructor for forward declares
    ~GridBuilder();

    /// @brief Main driving function
    /// @return `true` if successful
    bool update();

    /// @brief Accessor for the occ grid
    /// @return occupancy grid
    const nav_msgs::OccupancyGrid& getOccupancyGrid() const noexcept {return m_grid;}

    /// @brief Mutator for costmap data
    /// @param data The juicy, juicy data
    /// @{
    void setCostmapData(const CostmapData& data) noexcept {m_data = data;}
    void setCostmapData(CostmapData&& data)      noexcept {m_data = data;}
    /// @}

private:
    /// @brief Initializes the builder
    void initializeBuilder();

    /// @brief Builds the lookup table to be used for inflation
    void generateInflationLookupTable();

    /// @brief Resets the builder
    void resetBuilder();

    /// @brief Builds the occupancy grid
    /// @return `true` if successful
    bool buildGrid();

    /// @brief Marks occupied cells
    /// @param cloud The pointcloud to use to mark the cells occupied
    void markOccupiedCells(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /// @brief Inflates occupied cells
    void inflateOccupiedCells();

    std::shared_ptr<CostmapConfig>          m_cfg;             ///< Config of the costmap
    CostmapData                             m_data;            ///< Costmap data
    nav_msgs::OccupancyGrid                 m_grid;            ///< Occupancy grid to build
    tf::StampedTransform                    m_tf;              ///< Transform from whatever frame cloud is in to base_link
    std::unordered_set<std::size_t>         m_occopied_cells;  ///< Cells that are occupied
    InflationTable                          m_inflation_table; ///< Lookup table for inflated cell probabilities
};    

} // namespace cm

#endif // CM_GRID_BUILDER_HPP