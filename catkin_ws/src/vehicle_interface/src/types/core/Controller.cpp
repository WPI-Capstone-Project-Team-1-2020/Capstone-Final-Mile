// Component
#include "Controller.hpp"
#include "PIDController.hpp"

// Standard
#include <utility>

namespace vi
{

Controller::Controller(const std::shared_ptr<VehicleInterfaceConfig>& cfg) :
    m_cfg{cfg},
    m_linear_x_pid{std::make_unique<PIDController>(m_cfg->getLinearPIDConfig())},
    m_linear_y_pid{std::make_unique<PIDController>(m_cfg->getLinearPIDConfig())},
    m_angular_z_pid{std::make_unique<PIDController>(m_cfg->getAngularPIDConfig())}
{}

Controller::~Controller() = default;

void Controller::update(const ros::Time& now_s)
{
    m_linear_x_pid->setFeedback(m_current_state.linear.x);
    m_linear_y_pid->setFeedback(m_current_state.linear.y);
    m_angular_z_pid->setFeedback(m_current_state.angular.z);

    m_linear_x_pid->setCommandSetpoint(m_setpoint_cmd.linear.x);
    m_linear_y_pid->setCommandSetpoint(m_setpoint_cmd.linear.y);
    m_angular_z_pid->setCommandSetpoint(m_setpoint_cmd.angular.z);

    m_linear_x_pid->update(now_s);
    m_linear_y_pid->update(now_s);
    m_angular_z_pid->update(now_s);

    m_cmd.linear.x  = m_linear_x_pid->getOutput();
    m_cmd.linear.y  = m_linear_y_pid->getOutput();
    m_cmd.linear.z  = 0.0;
    m_cmd.angular.x = 0.0;
    m_cmd.angular.y = 0.0;
    m_cmd.angular.z = m_angular_z_pid->getOutput();
}



}// namespace vi