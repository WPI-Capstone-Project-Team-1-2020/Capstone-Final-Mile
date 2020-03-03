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

void ForwardSimHelper::forwardSimGraphNode(GraphNode& node, const float64_t time_step_ms)
{

    const float_t cur_x_m       = node.getEstimatedPointM().getX();
    const float_t cur_y_m       = node.getEstimatedPointM().getY();
    const float_t cur_heading_r = node.getEstimatedHeadingR();
    const float_t x_vel_mps     = node.getEstimatedLongitudinalVelocityMps() * std::cos(cur_heading_r);
    const float_t y_vel_mps     = node.getEstimatedLateralVelocityMps()      * std::sin(cur_heading_r);
    
}

} // namespace local_planner