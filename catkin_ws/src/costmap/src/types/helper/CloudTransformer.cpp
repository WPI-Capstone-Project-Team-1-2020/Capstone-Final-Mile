// Component
#include "CloudTransformer.hpp"
#include "RosConversionHelper.hpp"

// Libararies
#include <Eigen/Dense>

// Standard
#include <utility>

namespace cm
{

void CloudTransformer::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const geometry_msgs::Quaternion& q)
{
    Eigen::Matrix3d rot_mat = RosConversionHelper::quaternionMsgToRotationMatrix(q);
    Eigen::Matrix4d hom_tf;
    hom_tf << rot_mat (0, 0), rot_mat(0, 1), rot_mat(0, 2), 0.0,
              rot_mat (1, 0), rot_mat(1, 1), rot_mat(1, 2), 0.0,
              rot_mat (2, 0), rot_mat(2, 1), rot_mat(2, 2), 0.0,
              rot_mat (3, 0), rot_mat(3, 1), rot_mat(3, 2), 1.0;

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, hom_tf);
    *cloud = std::move(transformed_cloud);
}

} // namespace cm