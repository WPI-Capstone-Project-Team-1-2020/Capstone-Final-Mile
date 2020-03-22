#ifndef PLANNING_LOCAL_PLANNER_TOPIC_PUBLISHER_HPP
#define PLANNING_LOCAL_PLANNER_TOPIC_PUBLISHER_HPP

// Ros
#include <autonomy_msgs/Trajectory.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

// Standard
#include <memory>

// Forward declares
class LocalPlannerConfig;

namespace local_planner
{

/// @brief Class for publishing to topics
class TopicPublisher
{
public:
    /// @brief Default constructor
    /// @param nh nodehandle to subscribe to stuff and things with
    /// @param cfg config for the local planner
    TopicPublisher(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg);

    /// @brief Default destructor for forward declares
    ~TopicPublisher();

    /// @brief Publish the trajectory
    /// @param traj Trajectory to publish
    void publishTrajectory(const autonomy_msgs::Trajectory::ConstPtr& traj);

    /// @brief Publish the path for visualizing
    /// @param path The path to publish
    void publishPath(const nav_msgs::Path::ConstPtr& path);

private:

    std::shared_ptr<LocalPlannerConfig> m_cfg;      ///< Config of the local planner
    ros::Publisher                      m_traj_pub; ///< Trajectory publisher
    ros::Publisher                      m_path_pub; ///< Path publisher
};    

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_TOPIC_PUBLISHER_HPP