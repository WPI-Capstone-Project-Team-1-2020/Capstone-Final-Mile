
// Component
#include "CostmapConfig.hpp"
#include "TopicSubscriber.hpp"

namespace cm
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<CostmapConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getPointCloudTopic().empty() == false)
    {
        m_pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(m_cfg->getPointCloudTopic(), 1, &TopicSubscriber::onPointCloudReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
    
    if (m_cfg->getLocalPoseTopic().empty() == false)
    {
        m_pose_sub = nh.subscribe<nav_msgs::Odometry>(m_cfg->getLocalPoseTopic(), 1, &TopicSubscriber::onPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
}

TopicSubscriber::~TopicSubscriber() = default;

void TopicSubscriber::onPointCloudReceived(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    m_data.setPointCloud(msg);
}

void TopicSubscriber::onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_data.setLocalPose(msg);
}

} // namespace cm