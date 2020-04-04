
// Component
#include "VehicleInterfaceConfig.hpp"
#include "TopicSubscriber.hpp"

namespace vi
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<VehicleInterfaceConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getGoalReachedTopic().empty() == false)
    {
        m_goal_reached_sub = nh.subscribe<std_msgs::Bool>(m_cfg->getGoalReachedTopic(), 1, &TopicSubscriber::onGoalReachedReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getTakeoffReachedTopic().empty() == false)
    {
        m_takeoff_reached_sub = nh.subscribe<autonomy_msgs::GoalReached>(m_cfg->getTakeoffReachedTopic(), 1, &TopicSubscriber::onTakeoffGoalReachedReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getLandingReachedTopic().empty() == false)
    {
        m_landing_reached_sub = nh.subscribe<autonomy_msgs::GoalReached>(m_cfg->getLandingReachedTopic(), 1, &TopicSubscriber::onLandingGoalReachedReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getTrajectoryTopic().empty() == false)
    {
        m_traj_sub = nh.subscribe<autonomy_msgs::Trajectory>(m_cfg->getTrajectoryTopic(), 1, &TopicSubscriber::onTrajectoryReceived, this, ros::TransportHints().tcpNoDelay(true));
    }

    if (m_cfg->getTakeoffLandTopic().empty() == false)
    {
        m_takeoff_land_sub = nh.subscribe<geometry_msgs::Twist>(m_cfg->getTakeoffLandTopic(), 1, &TopicSubscriber::onTakeoffLandReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
    
    if (m_cfg->getLocalPoseTopic().empty() == false)
    {
        m_pose_sub = nh.subscribe<nav_msgs::Odometry>(m_cfg->getLocalPoseTopic(), 1, &TopicSubscriber::onPoseReceived, this, ros::TransportHints().tcpNoDelay(true));
    }
}

TopicSubscriber::~TopicSubscriber() = default;

void TopicSubscriber::onGoalReachedReceived(const std_msgs::Bool::ConstPtr& msg)
{
    m_data.setGoalReached(msg->data);
}

void TopicSubscriber::onTakeoffGoalReachedReceived(const autonomy_msgs::GoalReached::ConstPtr& msg)
{
    m_data.setTakeoffGoalReached(msg->goalReached);
}

void TopicSubscriber::onLandingGoalReachedReceived(const autonomy_msgs::GoalReached::ConstPtr& msg)
{
    m_data.setLandingGoalReached(msg->goalReached);
}

void TopicSubscriber::onTrajectoryReceived(const autonomy_msgs::Trajectory::ConstPtr& msg)
{    
    m_data.setTrajectory(msg);
}

void TopicSubscriber::onTakeoffLandReceived(const geometry_msgs::Twist::ConstPtr& msg)
{
    m_data.setTakeoffLandCommand(msg);
}

void TopicSubscriber::onPoseReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_data.setLocalPose(msg);
}

}// namespace vi