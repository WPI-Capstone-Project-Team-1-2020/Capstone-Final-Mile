#ifndef PLANNING_LOCAL_PLANNER_DATA_HPP
#define PLANNING_LOCAL_PLANNER_DATA_HPP

// Ros
#include <autonomy_msgs/GoalPose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

namespace local_planner
{

/// @brief Class for holding callback data for the local planner
class LocalPlannerData
{
public:
    /// @brief Accessor
    /// @return Val
    /// @{
    const nav_msgs::OccupancyGrid::ConstPtr& getCostmap()       const noexcept {return m_costmap;}
    const autonomy_msgs::GoalPose::ConstPtr& getGoalPose()      const noexcept {return m_goal_pose;}
    const nav_msgs::Odometry::ConstPtr&      getLocalPose()     const noexcept {return m_local_pose;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setCostmap(const nav_msgs::OccupancyGrid::ConstPtr& val)  noexcept {m_costmap = val;}
    void setGoalPose(const autonomy_msgs::GoalPose::ConstPtr& val) noexcept {m_goal_pose = val;}
    void setLocalPose(const nav_msgs::Odometry::ConstPtr& val)     noexcept {m_local_pose = val;}
    /// @}

private:
    nav_msgs::OccupancyGrid::ConstPtr       m_costmap;        ///< Costmap for the local planner
    autonomy_msgs::GoalPose::ConstPtr       m_goal_pose;      ///< Goal pose for the local planner
    nav_msgs::Odometry::ConstPtr            m_local_pose;     ///< Local pose for the local planner
};    

} // namespace local_planner

#endif // PLANNING_LOCAL_PLANNER_DATA_HPP