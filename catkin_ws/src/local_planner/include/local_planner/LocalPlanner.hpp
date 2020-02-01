// Component
#include "GraphNode.hpp"

// Ros
#include <ros/ros.h>

namespace planning
{

class LocalPlanner
{
public:
    LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~LocalPlanner() = default;

private:

};
}