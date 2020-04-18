
// Component
#include "CloudTransformer.hpp"
#include "CostmapConfig.hpp"
#include "GridBuilder.hpp"
#include "RosConversionHelper.hpp"

// Ros
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

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
    m_grid.info.height     = m_cfg->getCostmapHeightM()/m_cfg->getCostmapResolutionM();
    m_grid.info.width      = m_cfg->getCostmapWidthM()/m_cfg->getCostmapResolutionM();
    m_grid.info.resolution = m_cfg->getCostmapResolutionM();

    geometry_msgs::Pose origin;
    origin.position.x = -0.5*m_grid.info.width*m_cfg->getCostmapResolutionM();
    origin.position.y = -0.5*m_grid.info.height*m_cfg->getCostmapResolutionM();    
    
    m_grid.info.origin = std::move(origin);

    tf::TransformListener list;

    while (list.canTransform("camera_depth_optical_frame", "base_link", ros::Time(0)) == false)
    {
        continue;
    }

    try
    {
        list.lookupTransform("camera_depth_optical_frame", "base_link", ros::Time(0), m_tf);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_ERROR_STREAM(ex.what());
    }

    resetBuilder();

    ROS_INFO_STREAM("Generating Costmap Inflation Lookup Table... This may take a minute");

    generateInflationLookupTable();

    ROS_INFO_STREAM("Costmap Inflation Lookup Table Generated");
}

void GridBuilder::generateInflationLookupTable()
{
    auto get_grid_it = [&grid = this->m_grid](const float64_t x_m, const float64_t y_m) -> std::size_t
    {
        const std::size_t x = static_cast<std::size_t>((x_m - grid.info.origin.position.x)/grid.info.resolution);
        const std::size_t y = static_cast<std::size_t>((y_m - grid.info.origin.position.y)/grid.info.resolution);

        return (x + y*static_cast<std::size_t>(grid.info.width)); 
    };

    auto get_pt_m = [&grid = this->m_grid](const std::size_t grid_it) -> std::pair<float64_t, float64_t>
    {
        std::size_t height_pos = grid_it/grid.info.height;
        std::size_t width_pos  = grid_it%grid.info.height;
        float64_t x_m          = (static_cast<float64_t>(width_pos ) - 0.5*static_cast<float64_t>(grid.info.width ))*grid.info.resolution;
        float64_t y_m          = (static_cast<float64_t>(height_pos) - 0.5*static_cast<float64_t>(grid.info.height))*grid.info.resolution;

        return std::make_pair(x_m, y_m);
    };

    m_inflation_table.reserve(m_grid.data.size());

    constexpr float64_t angular_resolution_r{0.05};
    const     float64_t radial_resolution_m{m_grid.info.resolution};
    const     std::size_t num_angular_pts = static_cast<std::size_t>(2.0*M_PI/angular_resolution_r);
    const     std::size_t num_radial_pts  = static_cast<std::size_t>(m_cfg->getInflationRadiusM()/radial_resolution_m);    

    for (std::size_t grid_it = 0U; grid_it < m_grid.data.size(); ++grid_it)
    {
        InflationCluster cluster;

        float64_t x_m{0.0};
        float64_t y_m{0.0};
        std::tie(x_m, y_m) = get_pt_m(grid_it);

        for (std::size_t ang_it = 0U; ang_it < num_angular_pts; ++ang_it)
        {
            for (std::size_t rad_it = 0U; rad_it < num_radial_pts; ++rad_it)
            {
                const float64_t ang_r = static_cast<float64_t>(ang_it)/static_cast<float64_t>(num_angular_pts)*2.0*M_PI;
                const float64_t rad_m = static_cast<float64_t>(rad_it)/static_cast<float64_t>(num_radial_pts)*m_cfg->getInflationRadiusM();
                const float64_t x_i_m = x_m + rad_m*std::cos(ang_r); 
                const float64_t y_i_m = y_m + rad_m*std::sin(ang_r);

                const std::size_t inflated_grid_it = get_grid_it(x_i_m, y_i_m);
                if (inflated_grid_it < m_grid.data.size())
                {
                    const std::int8_t prob_occ = 100 - static_cast<std::int8_t>(rad_m/m_cfg->getInflationRadiusM()*100.0);
                    cluster.emplace_back(std::make_pair(inflated_grid_it, prob_occ));
                }
            }
        }

        m_inflation_table.emplace(grid_it, cluster);
    }
}

void GridBuilder::resetBuilder()
{
    m_occopied_cells.clear();

    m_grid.header.stamp = ros::Time::now();
    m_grid.data.clear();

    const std::size_t num_x_vals = static_cast<std::size_t>(m_grid.info.width);
    const std::size_t num_y_vals = static_cast<std::size_t>(m_grid.info.height);
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
    CloudTransformer::transformCloud(cloud, m_tf, m_data.getLocalPose()->pose.pose.orientation);
    CloudTransformer::trimCloud(cloud, m_cfg->getZBandpassM());

    markOccupiedCells(cloud);

    inflateOccupiedCells();

    return true;
}

void GridBuilder::markOccupiedCells(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    auto get_grid_it = [](const pcl::PointXYZ& pt, const nav_msgs::OccupancyGrid& grid) -> std::size_t
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

            if (grid_it >= grid.data.size())
            {
                return;
            }

            grid.data[grid_it] = 100;
            occupied_cells.emplace(grid_it);
        });
}

void GridBuilder::inflateOccupiedCells()
{
    std::for_each(m_occopied_cells.cbegin(),
        m_occopied_cells.cend(),
        [&table = this->m_inflation_table, &grid = this->m_grid](const std::size_t grid_it) -> void
        {
            const InflationCluster cluster = table.at(grid_it);
            std::for_each(cluster.cbegin(),
                cluster.cend(),
                [&grid](const std::pair<std::size_t, std::int8_t> loc_prob_pair) -> void
                {
                    std::size_t inflated_grid_it{0U};
                    std::int8_t prob_occ{0};
                    std::tie(inflated_grid_it, prob_occ) = loc_prob_pair;

                    if (grid.data[inflated_grid_it] < prob_occ)
                    {
                        grid.data[inflated_grid_it] = prob_occ;
                    }
                });
        });
}

} // namespace cm