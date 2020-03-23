// Component
#include "TrajectoryUnspooler.hpp"

// Standard
#include <utility>

namespace vi
{

TrajectoryUnspooler::~TrajectoryUnspooler() = default;

void TrajectoryUnspooler::update(const ros::Time& now_s)
{
    calculateCommandFromTrajectory(now_s);
}

void TrajectoryUnspooler::calculateCommandFromTrajectory(const ros::Time& now_s)
{
    if (m_data.getTrajectory()->execution_times.empty() == true)
    {
        ROS_ERROR_STREAM("Empty trajectory, falling out of the sky, you deserve this");

        return;
    }

    const autonomy_msgs::Trajectory::ConstPtr& traj = m_data.getTrajectory();

    std::size_t traj_it{0U};
    ros::Time traj_time = traj->execution_times[traj_it];
    const std::size_t num_cmds = traj->execution_times.size();    

    while (traj_time <= now_s)
    {
        if (traj_it >= num_cmds - 1U)
        {
            ROS_ERROR_STREAM("Trajectory does not contain a command near the current time, stopping");

            m_cmd.linear.x  = 0.0;
            m_cmd.linear.y  = 0.0;
            m_cmd.linear.z  = 0.0;
            m_cmd.angular.x = 0.0;
            m_cmd.angular.y = 0.0;
            m_cmd.angular.z = 0.0;

            return;
        }
        
        traj_time = traj->execution_times[traj_it];        

        traj_it++;        
    }

    if (traj_it - 2U < 0U)
    {
        ROS_ERROR_STREAM("Not enough commands in trajectory to interpolate, stopping");

        m_cmd.linear.x  = 0.0;
        m_cmd.linear.y  = 0.0;
        m_cmd.linear.z  = 0.0;
        m_cmd.angular.x = 0.0;
        m_cmd.angular.y = 0.0;
        m_cmd.angular.z = 0.0;

        return;
    }

    traj_it -= 2U;

    const ros::Time closest_time_s = traj->execution_times[traj_it];
    const geometry_msgs::Twist closest_cmd = traj->cmds[traj_it];
    const geometry_msgs::Twist next_cmd = traj->cmds[traj_it + 1U];
    const ros::Duration cmd_dt_s = traj->execution_times[traj_it + 1U] - closest_time_s;   


    float64_t x_acc_mps2     = (next_cmd.linear.x  - closest_cmd.linear.x)/cmd_dt_s.toSec();
    float64_t y_acc_mps2     = (next_cmd.linear.y  - closest_cmd.linear.y)/cmd_dt_s.toSec();
    float64_t z_ang_acc_rps2 = (next_cmd.angular.z - closest_cmd.angular.z)/cmd_dt_s.toSec();

    if (std::isnan(x_acc_mps2) == true)
    {
        x_acc_mps2 = 0.0;
    }

    if (std::isnan(y_acc_mps2) == true)
    {
        y_acc_mps2 = 0.0;
    }

    if (std::isnan(z_ang_acc_rps2) == true)
    {
        z_ang_acc_rps2 = 0.0;
    }    

    const ros::Duration dt_s = now_s - closest_time_s;    

    geometry_msgs::Twist cmd = closest_cmd;
    cmd.linear.x  += x_acc_mps2*dt_s.toSec();
    cmd.linear.y  += y_acc_mps2*dt_s.toSec();
    cmd.linear.z   = 0.0;
    cmd.angular.x  = 0.0;
    cmd.angular.y  = 0.0;
    cmd.angular.z += z_ang_acc_rps2*dt_s.toSec();

    m_cmd = std::move(cmd);   
}

}// namespace vi