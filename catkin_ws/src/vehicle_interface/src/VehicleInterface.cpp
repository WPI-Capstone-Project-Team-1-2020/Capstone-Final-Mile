// Component
#include "ForwardSimHelper.hpp"
#include "TopicPublisher.hpp"
#include "TopicSubscriber.hpp"
#include "VehicleInterface.hpp"
#include "VehicleInterfaceData.hpp"
#include "VehicleInterfaceConfig.hpp"

// Ros
#include <autonomy_msgs/Trajectory.h>

namespace vi
{

VehicleInterface::VehicleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh) :    
    m_cfg{std::make_shared<VehicleInterfaceConfig>(pnh)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)},
    m_topic_pub{std::make_unique<TopicPublisher>(nh, m_cfg)}
{
    m_timer = nh.createTimer(ros::Rate(m_cfg->getUpdateRateHz()), &VehicleInterface::update, this);
}

VehicleInterface::~VehicleInterface()
{
    ROS_INFO_STREAM("After I'm gone, your earth will be free to live out its miserable span of existence... As one of my satellites. And that's how it's going to be.");
}

void VehicleInterface::update(const ros::TimerEvent& event)
{
    if ((m_topic_sub->getVehicleInterfaceData().getTrajectory() != nullptr) &&
        (m_topic_sub->getVehicleInterfaceData().getLocalPose()  != nullptr))
    {
        VehicleInterfaceData data = m_topic_sub->getVehicleInterfaceData();
        data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getVehicleInterfaceData().getLocalPose(), event.current_real));
    }
}

} // namespace vi
