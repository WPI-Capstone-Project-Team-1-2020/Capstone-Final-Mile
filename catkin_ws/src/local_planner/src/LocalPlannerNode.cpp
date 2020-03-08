
#include <ros/ros.h>
#include <LocalPlanner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    local_planner::LocalPlanner planner(nh, pnh);

    while(ros::ok)
    {

        ros::spin();
    }

    return 0;

}
