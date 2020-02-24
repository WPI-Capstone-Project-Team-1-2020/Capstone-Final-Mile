// Component
#include "RosConversionHelper.hpp"
#include "RosConversionHelper.hpp"

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

float64_t RosConversionHelper::quaternionMsgToYawR(const geometry_msgs::Quaternion& q)
{
    tf::Quaternion new_q;
    tf::quaternionMsgToTF(q, new_q);
    float64_t roll_r, pitch_r, yaw_r;
    tf::Matrix3x3(std::move(new_q)).getRPY(roll_r, pitch_r, yaw_r);

    return yaw_r;
}

} // namespace local_planner