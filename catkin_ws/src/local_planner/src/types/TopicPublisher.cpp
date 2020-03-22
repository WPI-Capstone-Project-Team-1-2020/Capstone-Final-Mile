
// Component
#include "LocalPlannerConfig.hpp"
#include "TopicPublisher.hpp"

namespace local_planner
{

TopicPublisher::TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getTrajectoryTopic().empty() == false)
    {
        m_traj_pub = nh.advertise<autonomy_msgs::Trajectory>(m_cfg->getTrajectoryTopic(), 1);
    }

    if (m_cfg->getPathTopic().empty() == false)
    {
        m_path_pub = nh.advertise<nav_msgs::Path>(m_cfg->getPathTopic(), 1);
    }
}

TopicPublisher::~TopicPublisher() = default;

void TopicPublisher::publishTrajectory(const autonomy_msgs::Trajectory::ConstPtr& traj)
{
    m_traj_pub.publish(traj);
}

void TopicPublisher::publishPath(const nav_msgs::Path::ConstPtr& path)
{
    m_path_pub.publish(path);
}

}// namespace local_planner