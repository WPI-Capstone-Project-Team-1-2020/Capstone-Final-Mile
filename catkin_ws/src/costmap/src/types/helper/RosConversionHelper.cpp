// Component
#include "RosConversionHelper.hpp"

// Ros
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/shared_ptr.hpp>

// Standard
#include <utility>

namespace cm
{

pcl::PointCloud<pcl::PointXYZ>::Ptr RosConversionHelper::pointCloud2toPCL(const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc2, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> return_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, return_cloud);

    return boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(std::move(return_cloud));
}

Eigen::Matrix3d RosConversionHelper::quaternionMsgToRotationMatrix(const geometry_msgs::Quaternion& q)
{
    tf::Quaternion new_q;
    tf::quaternionMsgToTF(q, new_q);
    tf::Matrix3x3 tf_mat(new_q);

    float64_t roll_r;
    float64_t pitch_r;
    float64_t yaw_r;

    tf_mat.getRPY(roll_r, pitch_r, yaw_r);
    tf_mat.setRPY(roll_r, pitch_r, 0.0);

    Eigen::Matrix3d mat;
    tf::matrixTFToEigen(tf_mat, mat);

    return mat;
}

} // namespace cm