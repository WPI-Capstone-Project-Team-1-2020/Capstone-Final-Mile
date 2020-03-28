// Component
#include "Controller.hpp"
#include "ForwardSimHelper.hpp"
#include "TopicPublisher.hpp"
#include "TopicSubscriber.hpp"
#include "TrajectoryUnspooler.hpp"
#include "VehicleInterface.hpp"
#include "VehicleInterfaceData.hpp"
#include "VehicleInterfaceConfig.hpp"

// Ros
#include <autonomy_msgs/Trajectory.h>

namespace vi
{

VehicleInterface::VehicleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh) :    
    m_cfg{std::make_shared<VehicleInterfaceConfig>(pnh)},
    m_controller{std::make_unique<Controller>(m_cfg)},
    m_topic_sub{std::make_unique<TopicSubscriber>(nh, m_cfg)},
    m_topic_pub{std::make_unique<TopicPublisher>(nh, m_cfg)},
    m_unspooler{std::make_unique<TrajectoryUnspooler>()}
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

        if (data.getGoalReached() == true)
        {
            return;
        }

        data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getVehicleInterfaceData().getLocalPose(), event.current_real));
        m_unspooler->setVehicleInterfaceData(std::move(data));
        m_unspooler->update(event.current_real);
        m_controller->setCurrentState(data.getLocalPose()->twist.twist);
        m_controller->setCommandSetpoint(m_unspooler->getCommand());
        m_topic_pub->publishCommand(boost::make_shared<geometry_msgs::Twist>(m_controller->getCommand()));        
    }
}

} // namespace vi
