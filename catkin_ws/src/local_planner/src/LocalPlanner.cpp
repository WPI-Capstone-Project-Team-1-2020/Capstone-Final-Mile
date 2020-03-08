// Component
#include "AStar.hpp"
#include "ForwardSimHelper.hpp"
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

LocalPlanner::~LocalPlanner() = default;

void LocalPlanner::update(const ros::TimerEvent& event)
{
    if ((m_topic_sub->getLocalPlannerData().getGoalPose()  != nullptr) &&
        (m_topic_sub->getLocalPlannerData().getLocalPose() != nullptr))
    {
        LocalPlannerData data = m_topic_sub->getLocalPlannerData();
        data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getLocalPlannerData().getLocalPose(), event.current_real));
        m_solver->setLocalPlannerData(std::move(data));
        if (m_solver->update() == true)
        {

        }
        else
        {
            ROS_ERROR_STREAM("Unable to plan trajectory");
        }
    }
}

} // namespace local_planner
