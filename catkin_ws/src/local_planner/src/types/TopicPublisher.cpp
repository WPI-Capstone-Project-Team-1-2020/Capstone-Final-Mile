
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
}

TopicPublisher::~TopicPublisher() = default;

void TopicPublisher::publishTrajectory(const autonomy_msgs::Trajectory::ConstPtr& traj)
{
    m_traj_pub.publish(traj);
}

}// namespace local_planner