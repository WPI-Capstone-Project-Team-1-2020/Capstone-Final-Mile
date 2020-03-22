
#include <ros/ros.h>
#include <VehicleInterface.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_interface_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    //vi::VehicleInterface vi(nh, pnh);

    ros::spin();

    return 0;

}
