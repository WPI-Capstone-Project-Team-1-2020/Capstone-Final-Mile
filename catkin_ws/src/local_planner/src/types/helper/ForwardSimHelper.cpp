// Component
#include "ForwardSimHelper.hpp"

// Ros
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/shared_ptr.hpp>

// Standard
#include <utility>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

nav_msgs::Odometry::ConstPtr ForwardSimHelper::forwardSimPose(const nav_msgs::Odometry::ConstPtr& pose, const ros::Time& now_s)
{
    const ros::Duration dt_s = pose->header.stamp - now_s;

    nav_msgs::Odometry new_pose;

    new_pose.pose.pose.position.x = pose->pose.pose.position.x + pose->twist.twist.linear.x*dt_s.toSec();
    new_pose.pose.pose.position.y = pose->pose.pose.position.y + pose->twist.twist.linear.y*dt_s.toSec();
    new_pose.pose.pose.position.z = pose->pose.pose.position.z + pose->twist.twist.linear.z*dt_s.toSec();

    tf::Quaternion q;
    tf::quaternionMsgToTF(pose->pose.pose.orientation, q);
    float64_t roll_r, pitch_r, yaw_r;
    tf::Matrix3x3(std::move(q)).getRPY(roll_r, pitch_r, yaw_r);

    roll_r  += pose->twist.twist.angular.x*dt_s.toSec();
    pitch_r += pose->twist.twist.angular.y*dt_s.toSec();
    yaw_r   += pose->twist.twist.angular.z*dt_s.toSec();  
        
    geometry_msgs::Quaternion new_q;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll_r, pitch_r, yaw_r), new_q);
    new_pose.pose.pose.orientation = new_q;

    return boost::make_shared<nav_msgs::Odometry>(new_pose);
}

void ForwardSimHelper::forwardSimGraphNode(GraphNode& node, const GraphNode& parent_node, const float64_t time_step_ms)
{
    /// @TODO Incorporate Controller bias in estimated states
    const float64_t time_step_s = time_step_ms/1000.0;

    const float64_t cur_x_m = node.getEstimatedPointM().getX();
    const float64_t cur_y_m = node.getEstimatedPointM().getY();

    const float64_t cur_heading_r          = node.getEstimatedHeadingR();
    const float64_t cur_yaw_rate_rps       = node.getEstimatedYawRateRps();
    const float64_t cur_yaw_rate_rate_rpss = (cur_yaw_rate_rps - parent_node.getEstimatedYawRateRps())/time_step_s;
    const float64_t new_yaw_rate_rps       = cur_yaw_rate_rps + cur_yaw_rate_rate_rpss*time_step_s;
    const float64_t new_heading_r          = cur_heading_r + cur_yaw_rate_rps*time_step_s + cur_yaw_rate_rate_rpss*std::pow(time_step_s, 2U)/2.0;
    node.setCommandedYawRateRps(new_yaw_rate_rps);
    node.setEstimatedYawRateRps(new_yaw_rate_rps);
    node.setEstimatedHeadingR(new_heading_r);

    const float64_t cur_lon_vel_mps = node.getEstimatedLongitudinalVelocityMps();
    const float64_t cur_lat_vel_mps = node.getEstimatedLateralVelocityMps();
    const float64_t cur_x_vel_mps   = cur_lon_vel_mps*std::cos(cur_heading_r);
    const float64_t cur_y_vel_mps   = cur_lat_vel_mps*std::sin(cur_heading_r);

    const float64_t cur_x_acc_mpss = (cur_lon_vel_mps - parent_node.getEstimatedLongitudinalVelocityMps()) / 
                                      time_step_s*std::cos(cur_heading_r);
    const float64_t cur_y_acc_mpss = (cur_lat_vel_mps - parent_node.getEstimatedLateralVelocityMps()) / 
                                      time_step_s*std::sin(cur_heading_r);


    
    const float64_t new_x_vel_mps   = node.getEstimatedLongitudinalVelocityMps() * std::cos(cur_heading_r) + cur_x_acc_mpss*time_step_s;
    const float64_t new_y_vel_mps   = node.getEstimatedLateralVelocityMps()      * std::sin(cur_heading_r) + cur_y_acc_mpss*time_step_s;
    const float64_t new_lon_vel_mps = new_x_vel_mps/std::cos(cur_heading_r);
    const float64_t new_lat_vel_mps = new_y_vel_mps/std::sin(cur_heading_r);
    node.setCommandedLongitudinalVelocityMps(new_lon_vel_mps);
    node.setCommandedLateralVelocityMps(new_lat_vel_mps);
    node.setEstimatedLongitudinalVelocityMps(new_lon_vel_mps);
    node.setEstimatedLateralVelocityMps(new_lat_vel_mps);

    const float64_t new_x_m        = cur_x_m + cur_x_vel_mps*time_step_s + cur_x_acc_mpss*std::pow(time_step_s, 2U)/2.0;
    const float64_t new_y_m        = cur_y_m + cur_y_vel_mps*time_step_s + cur_y_acc_mpss*std::pow(time_step_s, 2U)/2.0;
    node.setEstimatedPointM(Point(new_x_m, new_y_m));
}

} // namespace local_planner