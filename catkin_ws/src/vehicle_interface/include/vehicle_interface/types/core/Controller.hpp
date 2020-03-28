#ifndef VI_CONTROLLER_HPP
#define VI_CONTROLLER_HPP

// Component
#include "VehicleInterfaceConfig.hpp"
#include "VehicleInterfaceData.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

// Ros
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Standard
#include <cstdint>
#include <memory>
#include <string>

namespace vi
{

// Forward Declares
class PIDController;

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for vehicle interface
class Controller
{
public:
   /// @brief Default constructor
   Controller(const std::shared_ptr<VehicleInterfaceConfig>& cfg);

   /// @brief Destructor for forward declares
   ~Controller();

   /// @brief Main driving function
   /// @param now_s Current program time
   void update(const ros::Time& now_s);

   /// @brief Accessor for command
   /// @return Command
   const geometry_msgs::Twist& getCommand() const noexcept {return m_cmd;}

   /// @brief Mutator for current state
   /// @param val Val
   void setCurrentState(const geometry_msgs::Twist& val) noexcept {m_current_state = val;}

   /// @brief Mutator for command setpoint
   /// @param val Val
   void setCommandSetpoint(const geometry_msgs::Twist& val) noexcept {m_setpoint_cmd = val;}

private:
    std::unique_ptr<PIDController> m_linear_x_pid;  ///< PID Controller on linear x
    std::unique_ptr<PIDController> m_linear_y_pid;  ///< PID Controller on linear y
    std::unique_ptr<PIDController> m_angular_z_pid; ///< PID Controller on angular z

    geometry_msgs::Twist m_setpoint_cmd;  ///< Setpoint
    geometry_msgs::Twist m_current_state; ///< Current state;
    geometry_msgs::Twist m_cmd;           ///< Output command

    std::shared_ptr<VehicleInterfaceConfig> m_cfg; ///< VI Config
};

} // namespace vi

#endif // VI_CONTROLLER_HPP
