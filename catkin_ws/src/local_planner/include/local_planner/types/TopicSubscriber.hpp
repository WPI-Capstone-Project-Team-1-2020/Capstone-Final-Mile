#ifndef PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP
#define PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP

// Ros
#include <ros/ros.h>

// Standard
#include <memory>

// Forward declares
class LocalPlannerConfig;

namespace local_planner
{

/// @brief Class for subscribing to topics
class TopicSubscriber
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the local planner
    TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicSubscriber();

private:
    std::shared_ptr<LocalPlannerConfig> m_cfg; ///< Config of the local planner
};    

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_TOPIC_SUBSCRIBER_HPP