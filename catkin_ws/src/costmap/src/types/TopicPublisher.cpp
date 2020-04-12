
// Component
#include "CostmapConfig.hpp"
#include "TopicPublisher.hpp"

namespace cm
{

TopicPublisher::TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<CostmapConfig> cfg) :
    m_cfg{cfg}
{
    if (m_cfg->getCostmapTopic().empty() == false)
    {
        m_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(m_cfg->getCostmapTopic(), 1);
    }

    if (m_cfg->getDiagnosticsTopic().empty() == false)
    {
        m_diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(m_cfg->getDiagnosticsTopic(), 1);
    }
}

TopicPublisher::~TopicPublisher() = default;

void TopicPublisher::publishCostmap(const nav_msgs::OccupancyGrid::ConstPtr& cm)
{
    m_costmap_pub.publish(cm);
}

void TopicPublisher::publishDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& statuses)
{
    m_diag_pub.publish(statuses);
}

}// namespace vi