// Component
#include "Costmap.hpp"
#include "CostmapData.hpp"
#include "CostmapConfig.hpp"
#include "ForwardSimHelper.hpp"
#include "GridBuilder.hpp"
#include "TopicPublisher.hpp"
#include "TopicSubscriber.hpp"

// Libraries
#include <boost/shared_ptr.hpp>

namespace cm
{

Costmap::Costmap(ros::NodeHandle& nh, ros::NodeHandle& pnh) :    
    m_cfg{std::make_shared<CostmapConfig>(pnh)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)},
    m_topic_pub{std::make_unique<TopicPublisher>(nh, m_cfg)},
    m_builder{std::make_unique<GridBuilder>(m_cfg)}
{
    m_timer = nh.createTimer(ros::Rate(m_cfg->getUpdateRateHz()), &Costmap::update, this);
}

Costmap::~Costmap()
{
    ROS_INFO_STREAM("After I'm gone, your earth will be free to live out its miserable span of existence... As one of my satellites. And that's how it's going to be.");
}

void Costmap::update(const ros::TimerEvent& event)
{
    if ((m_topic_sub->getCostmapData().getPointCloud() != nullptr) &&
        (m_topic_sub->getCostmapData().getLocalPose()  != nullptr))
    {        
        CostmapData data = m_topic_sub->getCostmapData();            
        data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getCostmapData().getLocalPose(), event.current_real));
        m_builder->setCostmapData(std::move(data));
        
        if (m_builder->update() == true)
        {
            m_topic_pub->publishCostmap(boost::make_shared<nav_msgs::OccupancyGrid>(m_builder->getOccupancyGrid()));

            updateDiagnostics(true);
        }
        else
        {
            ROS_ERROR_STREAM("Unable to Construct Occupancy Grid");

            updateDiagnostics(false);
        }
    }
    else
    {
        updateDiagnostics(false);
    }    
}

void Costmap::updateDiagnostics(const bool health)
{
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.level = health ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.name  = "Costmap Node";

    array.status.push_back(status);

    m_topic_pub->publishDiagnostics(boost::make_shared<diagnostic_msgs::DiagnosticArray>(array));
}

} // namespace vi
