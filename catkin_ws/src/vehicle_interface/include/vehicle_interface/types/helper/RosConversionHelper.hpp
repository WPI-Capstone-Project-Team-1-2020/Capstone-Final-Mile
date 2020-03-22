#ifndef VI_ROS_CONVERSION_HELPER_HPP
#define VI_ROS_CONVERSION_HELPER_HPP

// Ros
#include <geometry_msgs/Quaternion.h>

// Libraries
#include <boost/cstdfloat.hpp>

namespace vi
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class for integrating callback data to a given time
class RosConversionHelper
{
public:
    /// @brief Extracts a yaw in radians from a quaternion msg
    /// @param q Quaternion message to extract yaw from
    /// @return Yaw in radians
    static float64_t quaternionMsgToYawR(const geometry_msgs::Quaternion& q);
};    

} // namespace vi

#endif // VI_ROS_CONVERSION_HELPER_HPP