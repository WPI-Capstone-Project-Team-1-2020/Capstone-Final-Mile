#ifndef CM_CLOUD_TRANSFORMER_HPP
#define CM_CLOUD_TRANSFORMER_HPP

// Ros
#include <geometry_msgs/Quaternion.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

namespace cm
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for integrating callback data to a given time
class CloudTransformer
{
public:
    /// @brief Transforms pointcloud from base link to stabilized base link
    /// @param cloud OUTPUT - the cloud to transform
    /// @param q Quaternion of the cloud
    static void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const geometry_msgs::Quaternion& q);
};    

} // namespace cm

#endif // CM_CLOUD_TRANSFORMER_HPP