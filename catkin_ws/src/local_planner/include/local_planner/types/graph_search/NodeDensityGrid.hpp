#ifndef PLANNING_NODE_DENSITY_GRID_HPP
#define PLANNING_NODE_DENSITY_GRID_HPP

// Component
#include "GraphNode.hpp"
#include "NodeDensityConfig.hpp"
#include "Point.hpp"

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/functional/hash.hpp>

// Standard
#include <cstdint>
#include <unordered_map>

namespace local_planner
{

using float64_t    = boost::float64_t; ///< Alias for 64 bit float
using BinMap       = std::unordered_map<std::size_t, std::vector<GraphNode>>; ///< Alias for map of sets of nodes AKA bins

/// @brief Class to store information about a graph node in a graph search
class NodeDensityGrid
{
public:
    /// @brief Default constructor
    NodeDensityGrid(const NodeDensityConfig& cfg) :
        m_cfg{cfg}
    {};

    /// @brief Constructs a grid of bins around the origin
    /// @param origin The origin in cartesian space of the grid
    inline void setGridOrigin(const Point& origin)
    {
        m_open_node_bins.clear();
        std::size_t num_bins_per_row{static_cast<std::size_t>(m_cfg.getGridSizeM()/m_cfg.getBinSizeM())};
        std::size_t num_bins{static_cast<std::size_t>(std::pow(num_bins_per_row, 2U))};
        m_open_node_bins.reserve(num_bins);

        std::int64_t origin_x_m = static_cast<std::int64_t>(origin.getX());
        std::int64_t origin_y_m = static_cast<std::int64_t>(origin.getY());
        std::int64_t min_x_m    = origin_x_m - static_cast<std::int64_t>(m_cfg.getGridSizeM()/2.0);
        std::int64_t min_y_m    = origin_y_m - static_cast<std::int64_t>(m_cfg.getGridSizeM()/2.0);

        for (std::size_t bin_it = 0U; bin_it < num_bins; ++bin_it)
        {
            const std::int64_t bin_x_it  = bin_it%num_bins_per_row;
            const std::int64_t bin_y_it  = bin_it/num_bins_per_row;
            const float64_t    x_coord_m = static_cast<float64_t>(min_x_m) + static_cast<float64_t>(bin_x_it)*m_cfg.getBinSizeM();
            const float64_t    y_coord_m = static_cast<float64_t>(min_y_m) + static_cast<float64_t>(bin_y_it)*m_cfg.getBinSizeM();

            const std::size_t key = std::hash<Point>()(Point(x_coord_m, y_coord_m));
            m_open_node_bins.emplace(key, std::vector<GraphNode>());
            m_open_node_bins.at(key).reserve(m_cfg.getNumberNodesPerBin());
        }
    }

    /// @brief Adds a graph node to the open set
    /// @param node Node to be added    
    /// @return `true` if node was added
    inline bool addNodeToOpenSet(const GraphNode& node)
    {
        const std::size_t key = std::hash<Point>()(node.getEstimatedPointM());
        BinMap::iterator bin_it = m_open_node_bins.find(key);

        if (bin_it != m_open_node_bins.cend())
        {
            if (bin_it->second.size() < m_cfg.getNumberNodesPerBin())
            {
                bin_it->second.emplace_back(node);         

                return true;
            }

            std::make_heap(bin_it->second.begin(), bin_it->second.end(), std::greater<GraphNode>{});
            if (node.getCost() < bin_it->second.back().getCost())
            {
                std::pop_heap(bin_it->second.begin(), bin_it->second.end(), std::greater<GraphNode>{});
                bin_it->second.emplace_back(node);

                return true;
            }                
        }

        return false;
    }   

    /// @brief Removes a node from the open set
    /// @param node The node to remove
    inline void closeNode(const GraphNode& node)
    {
        const std::size_t key = std::hash<Point>()(node.getEstimatedPointM());
        BinMap::iterator bin_it = m_open_node_bins.find(key);
        if (bin_it != m_open_node_bins.cend())
        {
            std::vector<GraphNode>::const_iterator vec_it = std::find(bin_it->second.cbegin(), bin_it->second.cend(), node);
            m_open_node_bins.at(key).erase(vec_it);
        }
    } 

private:
    NodeDensityConfig  m_cfg;            ///< Config for this
    BinMap             m_open_node_bins; ///< Map of node bins
};

} // namespace local_planner

#endif // PLANNING_NODE_DENSITY_GRID_HPP
