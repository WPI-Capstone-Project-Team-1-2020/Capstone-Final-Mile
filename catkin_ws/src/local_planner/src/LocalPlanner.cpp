// Component
#include "LocalPlanner.hpp"
#include "LocalPlannerConfig.hpp"
#include "TopicSubscriber.hpp"

namespace local_planner
{

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    m_cfg{std::make_shared<LocalPlannerConfig>(pnh)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)}
{
    
}

LocalPlanner::~LocalPlanner(){}

}