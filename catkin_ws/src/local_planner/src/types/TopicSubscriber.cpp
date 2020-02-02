
// Component
#include "LocalPlannerConfig.hpp"
#include "TopicSubscriber.hpp"

namespace local_planner
{

TopicSubscriber::TopicSubscriber(ros::NodeHandle& nh, std::shared_ptr<LocalPlannerConfig> cfg) :
    m_cfg{cfg}
{

}

TopicSubscriber::~TopicSubscriber(){}


}// namespace local_planner