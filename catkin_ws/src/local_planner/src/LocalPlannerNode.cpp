
#include <ros/ros.h>
#include <LocalPlanner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}