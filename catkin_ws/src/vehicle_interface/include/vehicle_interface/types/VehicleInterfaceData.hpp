#ifndef VI_VEHICLE_INTERFACE_DATA_HPP
#define VI_VEHICLE_INTERFACE_DATA_HPP

// Ros
#include <autonomy_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>

namespace vi
{

/// @brief Class for holding callback data for the vehicle interface
class VehicleInterfaceData
{
public:
    /// @brief Accessor
    /// @return Val
    /// @{
    bool                                       getGoalReached() const noexcept {return m_goal_reached;}
    const autonomy_msgs::Trajectory::ConstPtr& getTrajectory()  const noexcept {return m_profile;}
    const nav_msgs::Odometry::ConstPtr&        getLocalPose()   const noexcept {return m_local_pose;}
    /// @}

    /// @brief Mutator
    /// @param val Val
    /// @{
    void setGoalReached(const bool val)                                noexcept {m_goal_reached = val;}
    void setTrajectory(const autonomy_msgs::Trajectory::ConstPtr& val) noexcept {m_profile = val;}
    void setLocalPose(const nav_msgs::Odometry::ConstPtr& val)         noexcept {m_local_pose = val;}
    /// @}

private:
    bool                                m_goal_reached; ///< True if goal reached
    autonomy_msgs::Trajectory::ConstPtr m_profile;      ///< Trajectory
    nav_msgs::Odometry::ConstPtr        m_local_pose;   ///< Local pose
};    

} // namespace vi

#endif // VI_VEHICLE_INTERFACE_DATA_HPP