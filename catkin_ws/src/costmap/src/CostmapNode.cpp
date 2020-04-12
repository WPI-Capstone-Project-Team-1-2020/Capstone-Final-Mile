// Component
#include "Costmap.hpp"

// Ros
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    cm::Costmap cm(nh, pnh);

    ros::spin();

    return 0;

}
