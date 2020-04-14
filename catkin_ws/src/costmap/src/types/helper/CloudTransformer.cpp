// Component
#include "CloudTransformer.hpp"
#include "RosConversionHelper.hpp"

// Libararies
#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>

// Standard
#include <utility>

namespace cm
{

void CloudTransformer::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::StampedTransform& tf, const geometry_msgs::Quaternion& q)
{
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(tf.getRotation(), q_);
    const Eigen::Matrix3d pc_rot_mat = RosConversionHelper::quaternionMsgToRotationMatrix(q_);

    const Eigen::Matrix3d rot_mat = RosConversionHelper::quaternionMsgToRotationMatrixIgnoreYaw(q).inverse()*pc_rot_mat.inverse();
    Eigen::Matrix4d hom_tf;
    hom_tf << rot_mat (0, 0), rot_mat(0, 1), rot_mat(0, 2), 0.0,
              rot_mat (1, 0), rot_mat(1, 1), rot_mat(1, 2), 0.0,
              rot_mat (2, 0), rot_mat(2, 1), rot_mat(2, 2), 0.0,
              rot_mat (3, 0), rot_mat(3, 1), rot_mat(3, 2), 1.0;

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, hom_tf);
    *cloud = std::move(transformed_cloud);
}

void CloudTransformer::trimCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float64_t band_m)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-band_m/2.0, band_m/2.0);    
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    pass.filter(filtered_cloud);
    *cloud = std::move(filtered_cloud);
}

} // namespace cm