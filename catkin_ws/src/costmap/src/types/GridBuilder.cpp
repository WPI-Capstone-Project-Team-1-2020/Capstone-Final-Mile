
// Component
#include "CloudTransformer.hpp"
#include "CostmapConfig.hpp"
#include "GridBuilder.hpp"
#include "RosConversionHelper.hpp"

// Ros
#include <geometry_msgs/Pose.h>

namespace cm
{

GridBuilder::GridBuilder(std::shared_ptr<CostmapConfig> cfg) :
    m_cfg{cfg}
{
    initializeBuilder();
}

GridBuilder::~GridBuilder() = default;

bool GridBuilder::update()
{
    resetBuilder();

    return buildGrid();
}

void GridBuilder::initializeBuilder()
{    
    m_grid.header.frame_id = "base_link";
    m_grid.info.height     = m_cfg->getCostmapHeightM();
    m_grid.info.width      = m_cfg->getCostmapWidthM();
    m_grid.info.resolution = m_cfg->getCostmapResolutionM();

    geometry_msgs::Pose origin;
    origin.position.x = m_grid.info.width/2.0;
    origin.position.y = m_grid.info.height/2.0;    
    
    m_grid.info.origin = std::move(origin);
}

void GridBuilder::resetBuilder()
{
    m_occopied_cells.clear();

    m_grid.header.stamp = ros::Time::now();
    m_grid.data.clear();

    const std::size_t num_x_vals = static_cast<std::size_t>(m_grid.info.width/m_grid.info.resolution);
    const std::size_t num_y_vals = static_cast<std::size_t>(m_grid.info.height/m_grid.info.resolution);
    const std::size_t num_vals   = num_x_vals*num_y_vals;
    m_grid.data.reserve(num_vals);

    for (std::size_t data_it = 0U; data_it < num_vals; ++data_it)
    {
        m_grid.data.emplace_back(0);
    }
}

bool GridBuilder::buildGrid()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = RosConversionHelper::pointCloud2toPCL(m_data.getPointCloud());
    CloudTransformer::transformCloud(cloud, m_data.getLocalPose()->pose.pose.orientation);
    CloudTransformer::trimCloud(cloud, m_cfg->getZBandpassM());

    markOccupiedCells(cloud);

    inflateOccupiedCells();

    return true;
}

void GridBuilder::markOccupiedCells(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    auto get_grid_it = [](const pcl::PointXYZ& pt, const nav_msgs::OccupancyGrid& grid)
    {
        const std::size_t x = static_cast<std::size_t>((pt.x - grid.info.origin.position.x)/grid.info.resolution);
        const std::size_t y = static_cast<std::size_t>((pt.y - grid.info.origin.position.y)/grid.info.resolution);

        return (x + y*static_cast<std::size_t>(grid.info.width)); 
    };

    std::for_each(cloud->begin(),
        cloud->end(),
        [&grid = this->m_grid, &occupied_cells = this->m_occopied_cells, &get_grid_it](const pcl::PointXYZ& pt) -> void
        {
            const std::size_t grid_it = get_grid_it(pt, grid);
            grid.data[grid_it] = 100;
            occupied_cells.emplace(grid_it);
        });
}

void GridBuilder::inflateOccupiedCells()
{
    for (const auto& grid_it : m_occopied_cells)
    {
        
    }
}

} // namespace cm