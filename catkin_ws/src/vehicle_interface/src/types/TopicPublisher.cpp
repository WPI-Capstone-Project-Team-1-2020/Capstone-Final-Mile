
// Component
#include "VehicleInterfaceConfig.hpp"
#include "TopicPublisher.hpp"

namespace vi
{

TopicPublisher::TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<VehicleInterfaceConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getCommandTopic().empty() == false)
    {
        m_traj_pub = nh.advertise<geometry_msgs::Twist>(m_cfg->getCommandTopic(), 1);
    }

    if (m_cfg->getDiagnosticsTopic().empty() == false)
    {
        m_diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(m_cfg->getDiagnosticsTopic(), 1);
    }
}

TopicPublisher::~TopicPublisher() = default;

void TopicPublisher::publishCommand(const geometry_msgs::Twist::ConstPtr& cmd)
{
    m_traj_pub.publish(cmd);
}

void TopicPublisher::publishDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& statuses)
{
    m_diag_pub.publish(statuses);
}

}// namespace vi