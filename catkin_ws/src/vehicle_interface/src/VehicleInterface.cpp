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
    bool pass_through{false};

    if (m_topic_sub->getVehicleInterfaceData().getTakeoffGoalReached() == true)
    {        
        if ((m_topic_sub->getVehicleInterfaceData().getTrajectory() != nullptr) &&
            (m_topic_sub->getVehicleInterfaceData().getLocalPose()  != nullptr))
        {
            VehicleInterfaceData data = m_topic_sub->getVehicleInterfaceData();

            if (data.getGoalReached() == true)
            {
                if (data.getLandingGoalReached() == false)
                {
                    pass_through = true;
                }
            }

            if (pass_through == false)
            {
                data.setLocalPose(ForwardSimHelper::forwardSimPose(m_topic_sub->getVehicleInterfaceData().getLocalPose(), event.current_real));
                m_unspooler->setVehicleInterfaceData(std::move(data));
                m_unspooler->update(event.current_real);
                m_controller->setCurrentState(data.getLocalPose()->twist.twist);
                m_controller->setCommandSetpoint(m_unspooler->getCommand());
                m_controller->update(event.current_real);
                m_topic_pub->publishCommand(boost::make_shared<geometry_msgs::Twist>(m_controller->getCommand()));       
            }
        }            
    }
    
    if (pass_through == false)
    {
        if (m_topic_sub->getVehicleInterfaceData().getGoalReached() == true)
        {
            if ((m_topic_sub->getVehicleInterfaceData().getTakeoffGoalReached() == false) ||
                (m_topic_sub->getVehicleInterfaceData().getLandingGoalReached() == false))
            {
                pass_through = true;                            
            }
        }
    }

    if ((m_topic_sub->getVehicleInterfaceData().getTakeoffGoalReached() == false) &&
        (m_topic_sub->getVehicleInterfaceData().getLandingGoalReached() == false) &&
        (m_topic_sub->getVehicleInterfaceData().getGoalReached() == false))
        {
            pass_through = true;
        }

    if (pass_through == true)
    {        
        if (m_topic_sub->getVehicleInterfaceData().getTakeoffLandCommand() != nullptr)
        {            
            m_topic_pub->publishCommand(m_topic_sub->getVehicleInterfaceData().getTakeoffLandCommand());
        }
        else
        {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = 0.0;
            m_topic_pub->publishCommand(boost::make_shared<geometry_msgs::Twist>(cmd));
        }
    }

    if (m_topic_sub->getVehicleInterfaceData().getLocalPose()  == nullptr)
    {
        updateDiagnostics(false);
    }
    else
    {
        updateDiagnostics(true);
    }
    

}

void VehicleInterface::updateDiagnostics(const bool health)
{
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.level = health ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.name  = "Vehicle Interface Node";

    array.status.push_back(status);

    m_topic_pub->publishDiagnostics(boost::make_shared<diagnostic_msgs::DiagnosticArray>(array));
}

} // namespace vi
