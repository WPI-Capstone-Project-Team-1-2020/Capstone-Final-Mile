
// Component
#include "VehicleInterfaceConfig.hpp"
#include "TopicSubscriber.hpp"

namespace vi
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<VehicleInterfaceConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getTrajectoryTopic().empty() == false)
    {
        m_traj_sub = nh.subscribe<autonomy_msgs::Trajectory>(m_cfg->getTrajectoryTopic(), 1, &TopicSubscriber::onTrajectoryReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getLocalPoseTopic().empty() == false)
    {
        m_pose_sub = nh.subscribe<nav_msgs::Odometry>(m_cfg->getLocalPoseTopic(), 1, &TopicSubscriber::onPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
}

TopicSubscriber::~TopicSubscriber() = default;

void TopicSubscriber::onTrajectoryReceived(const autonomy_msgs::Trajectory::ConstPtr& msg)
{
    m_data.setTrajectory(msg);
}

void TopicSubscriber::onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_data.setLocalPose(msg);
}

}// namespace vi