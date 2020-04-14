#ifndef CM_CLOUD_TRANSFORMER_HPP
#define CM_CLOUD_TRANSFORMER_HPP

// Ros
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

namespace cm
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for transforming a pointcloud
class CloudTransformer
{
public:
    /// @brief Transforms pointcloud from base link to stabilized base link
    /// @param cloud OUTPUT - the cloud to transform
    /// @param tf The transform from cloud frame to base_link
    /// @param q Quaternion of the cloud
    static void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::StampedTransform& tf, const geometry_msgs::Quaternion& q);

    /// @brief Trims a cloud to a banded FOV in z-axis
    /// @param cloud OUTPUT - the cloud to band
    /// @param band_m Band in meters to trim the cloud (z-axis)
    static void trimCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float64_t band_m);
};    

} // namespace cm

#endif // CM_CLOUD_TRANSFORMER_HPP