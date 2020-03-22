// Component
#include "AStar.hpp"
#include "ForwardSimHelper.hpp"
#include "LocalPlanner.hpp"
#include "LocalPlannerConfig.hpp"
#include "TopicPublisher.hpp"
#include "TopicSubscriber.hpp"
#include "TrajectorySolver.hpp"

// Ros
#include <autonomy_msgs/Trajectory.h>

namespace local_planner
{

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) :    
    m_cfg{std::make_shared<LocalPlannerConfig>(pnh)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)},
    m_topic_pub{std::make_unique<TopicPublisher>(nh, m_cfg)},
    m_solver{std::make_unique<AStar>(m_cfg)},
    m_traj_solver{std::make_unique<TrajectorySolver>(m_cfg->getTrajectoryConfig())}
{
    m_timer = nh.createTimer(ros::Rate(m_cfg->getUpdateRateHz()), &LocalPlanner::update, this);
}

LocalPlanner::~LocalPlanner()
{
    ROS_INFO_STREAM("After I'm gone, your earth will be free to live out its miserable span of existence... As one of my satellites. And that's how it's going to be.");
}

void LocalPlanner::update(const ros::TimerEvent& event)
{
    if ((m_topic_sub->getLocalPlannerData().getGoalPose()  != nullptr) &&
        (m_topic_sub->getLocalPlannerData().getLocalPose() != nullptr))
    {
        LocalPlannerData data = m_topic_sub->getLocalPlannerData();
        data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getLocalPlannerData().getLocalPose(), event.current_real));
        m_solver->setLocalPlannerData(data);
        if (m_solver->update() == true)
        {
            m_traj_solver->setLocalPlannerData(std::move(data));
            const autonomy_msgs::Trajectory::ConstPtr& traj = m_traj_solver->calculateTrajectory(m_solver->getPath(), event.current_real);
            const nav_msgs::Path::ConstPtr ros_path = m_solver->getRosPath();
            m_topic_pub->publishTrajectory(traj);
            m_topic_pub->publishPath(ros_path);
        }
        else
        {
            ROS_ERROR_STREAM("Unable to plan trajectory");
        }
    }
}

} // namespace local_planner
