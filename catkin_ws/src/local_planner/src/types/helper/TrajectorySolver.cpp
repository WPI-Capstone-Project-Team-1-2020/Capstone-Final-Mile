// Component
#include "RosConversionHelper.hpp"
#include "TrajectorySolver.hpp"

// Ros
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/shared_ptr.hpp>
#include <unsupported/Eigen/Polynomials>
#include <unsupported/Eigen/Splines>

// Standard
#include <utility>

namespace local_planner
{

TrajectorySolver::TrajectorySolver(const TrajectoryConfig& cfg) :
    m_cfg{cfg}
{}

TrajectorySolver::~TrajectorySolver() = default;

autonomy_msgs::Trajectory::ConstPtr TrajectorySolver::calculateTrajectory(const std::vector<Point>& path, const ros::Time& now_s)
{
    m_dist_based_traj.cmds.clear();
    m_dist_based_traj.cmds.reserve(path.size());
    m_dist_based_traj.execution_times.clear();
    m_dist_based_traj.execution_times.reserve(path.size());
    m_time_based_traj.cmds.clear();
    m_time_based_traj.execution_times.clear();

    calculateDistanceBasedTrajectory(path);

    calculateTimeBasedTrajectory(now_s);    

    return boost::make_shared<autonomy_msgs::Trajectory>(m_time_based_traj);
}

void TrajectorySolver::calculateDistanceBasedTrajectory(const std::vector<Point>& path)
{
    if (path.size() < 2U)
    {
        return;
    }
    
    Point previous_point = path.front();
    float64_t total_path_dist_m{0.0};

    for (const auto& pt : path)
    {
        if (pt == previous_point)
        {
            previous_point = pt;

            continue;
        }

        total_path_dist_m += std::sqrt(std::pow(pt.getX() - previous_point.getX(), 2U) + std::pow(pt.getY() - previous_point.getY(), 2U));

        previous_point = pt;
    }


    const float64_t end_speed_mps = m_data.getGoalPose()->speed_mps;
    geometry_msgs::Twist current_state = m_data.getLocalPose()->twist.twist;
    float64_t current_heading_r = RosConversionHelper::quaternionMsgToYawR(m_data.getLocalPose()->pose.pose.orientation);        
    correctAngle(current_heading_r);
    float64_t dist_traveled_m{0.0};   
    float64_t total_time_s{0.0}; 

    for (std::vector<Point>::const_iterator path_it = path.cbegin(); path_it != path.cend() - 1U; ++path_it)
    {
        const std::vector<Point>::const_iterator next_it = path_it + 1U;
        const float64_t dx_m = next_it->getX() - path_it->getX();
        const float64_t dy_m = next_it->getY() - path_it->getY();
        const float64_t dp_m = std::sqrt(std::pow(dx_m, 2U) + std::pow(dy_m, 2U));        
        dist_traveled_m += dp_m;

        const float64_t cur_vel_x = current_state.linear.x*std::cos(current_heading_r);
        const float64_t cur_vel_y = current_state.linear.y*std::sin(current_heading_r);
        const float64_t cur_speed_mps = std::sqrt(std::pow(cur_vel_x, 2U) + std::pow(cur_vel_y, 2U));

        const float64_t dist_left_m = total_path_dist_m - dist_traveled_m;

        Eigen::PolynomialSolver<float64_t, Eigen::Dynamic> solver;
        Eigen::VectorXd coeff(3U);

        coeff << 0.0, cur_speed_mps, -m_cfg.getMaxAccelMps2();
        solver.compute(coeff);
        bool has_real{true};
        const float64_t time_to_stop_s = solver.greatestRealRoot(has_real);

        if (has_real == false)
        {
            ROS_ERROR_STREAM("No real roots found for slowing trajectory polynomial fitting");

            return;
        }
        
        const float64_t dist_to_stop   = cur_speed_mps*time_to_stop_s - m_cfg.getMaxAccelMps2()*std::pow(time_to_stop_s, 2U);

        bool needs_to_slow = dist_to_stop >= dist_left_m;

        coeff = Eigen::VectorXd(3U);
        coeff << -dp_m, cur_speed_mps, m_cfg.getMaxAccelMps2();
        solver.compute(coeff);
        float64_t dt_s = solver.greatestRealRoot(has_real);

        if (has_real == false)
        {
            ROS_ERROR_STREAM("No real roots found for trajectory polynomial fitting");

            return;
        }

        const float64_t next_speed_mps = needs_to_slow ? std::max(cur_speed_mps - m_cfg.getMaxAccelMps2()*dt_s, 0.0) : cur_speed_mps + m_cfg.getMaxAccelMps2()*dt_s;

        if (next_speed_mps > m_cfg.getMaxSpeedMps())
        {
            const float64_t dt_to_max_vel_s = (m_cfg.getMaxSpeedMps() - cur_speed_mps)/m_cfg.getMaxAccelMps2();
            const float64_t dp_to_max_vel_m = cur_speed_mps*dt_to_max_vel_s + m_cfg.getMaxAccelMps2()*std::pow(dt_to_max_vel_s, 2U)/2.0;
            const float64_t dp_to_next_pt_m = dp_m - dp_to_max_vel_m;
            const float64_t dt_to_next_pt_s = dp_to_next_pt_m/m_cfg.getMaxSpeedMps();
            dt_s = dt_to_max_vel_s + dt_to_next_pt_s;
        }

        const float64_t commanded_speed_mps = std::min(next_speed_mps, m_cfg.getMaxSpeedMps());
        float64_t velocity_heading_r  = std::atan2(dy_m, dy_m);
        correctAngle(velocity_heading_r);
        float64_t actual_vs_velocity_heading_diff_r = current_heading_r - velocity_heading_r;
        correctAngle(actual_vs_velocity_heading_diff_r);
        if (actual_vs_velocity_heading_diff_r > M_PI)
        {
            actual_vs_velocity_heading_diff_r -= 2.0*M_PI;
        }

        const float64_t lon_vel_mps = commanded_speed_mps*std::cos(actual_vs_velocity_heading_diff_r);
        const float64_t lat_vel_mps = commanded_speed_mps*std::sin(actual_vs_velocity_heading_diff_r);
                
        current_state.linear.x = lon_vel_mps;
        current_state.linear.y = lat_vel_mps;

        float64_t d_heading_r = velocity_heading_r - current_heading_r;
        correctAngle(d_heading_r);

        if (d_heading_r > M_PI)
        {
            d_heading_r -= 2.0*M_PI;
        }

        float64_t yaw_rate_rps = d_heading_r/dt_s;
        yaw_rate_rps = yaw_rate_rps < -m_cfg.getMaxYawRateRps() ? -m_cfg.getMaxYawRateRps() : yaw_rate_rps;
        yaw_rate_rps = yaw_rate_rps >  m_cfg.getMaxYawRateRps() ?  m_cfg.getMaxYawRateRps() : yaw_rate_rps;

        current_heading_r += yaw_rate_rps*dt_s;
        correctAngle(current_heading_r);

        current_state.angular.z = yaw_rate_rps;

        m_dist_based_traj.cmds.emplace_back(current_state);
        
        m_dist_based_traj.execution_times.emplace_back(ros::Time(total_time_s));

        total_time_s += dt_s;
    }
}

void TrajectorySolver::calculateTimeBasedTrajectory(const ros::Time& now_s)
{
    if (m_dist_based_traj.cmds.size() != m_dist_based_traj.execution_times.size() || m_dist_based_traj.cmds.size() == 0U)
    {
        ROS_ERROR_STREAM("Bad command formed in distance based trajectory calculation");

        return;
    }    

    using Spline1d        = Eigen::Spline<float64_t, 2>;
    using Spline1dFitting = Eigen::SplineFitting<Spline1d>;

    Eigen::MatrixXd x_vel_points(2, m_dist_based_traj.execution_times.size());
    Eigen::MatrixXd y_vel_points(2, m_dist_based_traj.execution_times.size());
    Eigen::MatrixXd yaw_rate_points(2, m_dist_based_traj.execution_times.size());

    float64_t prev_time_s = m_dist_based_traj.execution_times.front().toSec();
    float64_t total_traj_time_s = 0.0;

    for (std::size_t cmd_it = 0U; cmd_it < m_dist_based_traj.execution_times.size(); ++cmd_it)
    {
        const float64_t current_time_s = m_dist_based_traj.execution_times[cmd_it].toSec();
        total_traj_time_s += current_time_s - prev_time_s;
        prev_time_s = current_time_s;
        
        x_vel_points(0, cmd_it)    = current_time_s;
        y_vel_points(0, cmd_it)    = current_time_s;
        yaw_rate_points(0, cmd_it) = current_time_s;
    }

    for (std::size_t cmd_it = 0U; cmd_it < m_dist_based_traj.cmds.size(); ++cmd_it)
    {        
        x_vel_points(1, cmd_it)    = m_dist_based_traj.cmds[cmd_it].linear.x;
        y_vel_points(1, cmd_it)    = m_dist_based_traj.cmds[cmd_it].linear.y;
        yaw_rate_points(1, cmd_it) = m_dist_based_traj.cmds[cmd_it].angular.z;        
    }    

    const Spline1d x_vel_spline    = Spline1dFitting::Interpolate(x_vel_points,    3U);
    const Spline1d y_vel_spline    = Spline1dFitting::Interpolate(y_vel_points,    3U);
    const Spline1d yaw_rate_spline = Spline1dFitting::Interpolate(yaw_rate_points, 3U);    

    const float64_t spline_discretization = (m_cfg.getTimeStepMs()/1000.0)/total_traj_time_s;        
    const std::size_t num_spline_pts = static_cast<std::size_t>(1.0/spline_discretization);    

    for (std::size_t spline_it = 0U; spline_it < num_spline_pts; ++spline_it)
    {
        Eigen::MatrixXd x_vel_pt    = x_vel_spline(static_cast<float64_t>(spline_it)*spline_discretization);
        Eigen::MatrixXd y_vel_pt    = y_vel_spline(static_cast<float64_t>(spline_it)*spline_discretization);
        Eigen::MatrixXd yaw_rate_pt = yaw_rate_spline(static_cast<float64_t>(spline_it)*spline_discretization);        

        geometry_msgs::Twist current_state;
        current_state.linear.x  = x_vel_pt(1);
        current_state.linear.y  = y_vel_pt(1);        
        current_state.angular.z = yaw_rate_pt(1);

        m_time_based_traj.cmds.emplace_back(current_state);
        m_time_based_traj.execution_times.emplace_back(now_s + ros::Duration(x_vel_pt(0)));        
    }    
}

void TrajectorySolver::correctAngle(float64_t& angle)
{
    if (angle > 2.0*M_PI)
    {
        angle -= 2.0*M_PI;
    }
    if (angle < 0)
    {
        angle += 2*M_PI;
    }
}

} // namespace local_planner
