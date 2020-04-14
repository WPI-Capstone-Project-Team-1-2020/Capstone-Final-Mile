#ifndef CM_ROS_CONVERSION_HELPER_HPP
#define CM_ROS_CONVERSION_HELPER_HPP

// Ros
#include <geometry_msgs/Quaternion.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <Eigen/Dense>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

namespace cm
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for integrating callback data to a given time
class RosConversionHelper
{
public:
    /// @brief Converts ros pc2 to pcl pointcloud
    /// @param pc2 PointCloud2 msg
    /// @return pcl PC
    static pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2toPCL(const sensor_msgs::PointCloud2::ConstPtr& pc2);

    /// @brief Extracts a rotation matrix from quaternion
    /// @param q Quaternion message to extract yaw from
    /// @return Rotation Matrix
    static Eigen::Matrix3d quaternionMsgToRotationMatrix(const geometry_msgs::Quaternion& q);

    /// @brief Extracts a rotation matrix from quaternion, ignores yaw
    /// @param q Quaternion message to extract yaw from
    /// @return Rotation Matrix
    static Eigen::Matrix3d quaternionMsgToRotationMatrixIgnoreYaw(const geometry_msgs::Quaternion& q);
};    

} // namespace cm

#endif // CM_ROS_CONVERSION_HELPER_HPP