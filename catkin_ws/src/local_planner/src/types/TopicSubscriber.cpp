
// Component
#include "LocalPlannerConfig.hpp"
#include "TopicSubscriber.hpp"

namespace local_planner
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getCostmapTopic().empty() == false)
    {
        m_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>(m_cfg->getCostmapTopic(), 1, &TopicSubscriber::onCostmapReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getGoalTopic().empty() == false)
    {
        m_goal_sub = nh.subscribe<autonomy_msgs::GoalPose>(m_cfg->getGoalTopic(), 1, &TopicSubscriber::onGoalPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getLocalPoseTopic().empty() == false)
    {
        m_pose_sub = nh.subscribe<nav_msgs::Odometry>(m_cfg->getLocalPoseTopic(), 1, &TopicSubscriber::onPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
}

TopicSubscriber::~TopicSubscriber() = default;

void TopicSubscriber::onCostmapReceived(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    m_data.setCostmap(msg);    
}

void TopicSubscriber::onGoalPoseReceived(const autonomy_msgs::GoalPose::ConstPtr& msg)
{
    m_data.setGoalPose(msg);
}

void TopicSubscriber::onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_data.setLocalPose(msg);
}

}// namespace local_planner