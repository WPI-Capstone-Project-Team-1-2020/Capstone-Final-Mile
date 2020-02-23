// Component
#include "AStar.hpp"
#include "LocalPlanner.hpp"
#include "LocalPlannerConfig.hpp"
#include "TopicSubscriber.hpp"

namespace local_planner
{

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) :    
    m_cfg{std::make_shared<LocalPlannerConfig>(pnh)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)},
    m_solver{std::make_unique<AStar>(m_cfg)}
{
    m_timer = nh.createTimer(ros::Rate(m_cfg->getUpdateRateHz()), &LocalPlanner::update, this);
}

void LocalPlanner::update(const ros::TimerEvent& event)
{
    m_solver->setLocalPlannerData(m_topic_sub->getLocalPlannerData());
    if (m_solver->update() == true)
    {

    }
    else
    {
        ROS_ERROR_STREAM("Unable to plan trajectory");
    }
}

}