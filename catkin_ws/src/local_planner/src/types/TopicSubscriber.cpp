
// Component
#include "LocalPlannerConfig.hpp"
#include "TopicSubscriber.hpp"

namespace local_planner
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getLocalPoseTopic().empty() == false)
    {
        m_pose_sub = nh.subscribe<nav_msgs::Odometry>(m_cfg->getLocalPoseTopic(), 1, &TopicSubscriber::onPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
}

TopicSubscriber::~TopicSubscriber(){}

void TopicSubscriber::onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg)
{

}

}// namespace local_planner