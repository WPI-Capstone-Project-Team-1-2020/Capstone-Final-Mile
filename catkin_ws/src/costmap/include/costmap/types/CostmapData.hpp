#ifndef CM_COSTMAP_DATA_HPP
#define CM_COSTMAP_DATA_HPP

// Ros
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

namespace cm
{

/// @brief Class for holding callback data for the costmap
class CostmapData
{
public:
    /// @brief Accessor
    /// @return Val
    /// @{
    const sensor_msgs::PointCloud2::ConstPtr& getPointCloud() const noexcept {return m_pc;}
    const nav_msgs::Odometry::ConstPtr&       getLocalPose()  const noexcept {return m_local_pose;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setPointCloud(const sensor_msgs::PointCloud2::ConstPtr& val) noexcept {m_pc = val;}
    void setLocalPose(const nav_msgs::Odometry::ConstPtr& val)        noexcept {m_local_pose = val;}
    /// @}

private:
    sensor_msgs::PointCloud2::ConstPtr  m_pc;                     ///< Pointcloud
    nav_msgs::Odometry::ConstPtr        m_local_pose;             ///< Local pose
};    

} // namespace cm

#endif // CM_COSTMAP_DATA_HPP