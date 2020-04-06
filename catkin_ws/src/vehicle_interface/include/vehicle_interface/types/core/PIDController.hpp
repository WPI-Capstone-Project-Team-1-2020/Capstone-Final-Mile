#ifndef VI_PID_CONTROLLER_HPP
#define VI_PID_CONTROLLER_HPP

// Component
#include "PIDConfig.hpp"
#include "VehicleInterfaceData.hpp"

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/circular_buffer.hpp>

// Ros
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Standard
#include <cstdint>
#include <string>

namespace vi
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store config values for vehicle interface
class PIDController
{
public:
   /// @brief Default constructor
   explicit PIDController(const PIDConfig& cfg);

   /// @brief Destructor for forward declares
   ~PIDController();

   /// @brief Main driving function
   /// @param now_s Current program time
   void update(const ros::Time& now_s);

   /// @brief Accessor for command
   /// @return Command
   float64_t getOutput() const noexcept {return m_output;}

   /// @brief Mutator
   /// @param val Val
   /// @{
   void setFeedback(const float64_t val)        noexcept {m_feedback = val;}
   void setCommandSetpoint(const float64_t val) noexcept {m_setpoint = val;}
   /// @}

private:
    /// @brief Helper function to update the delta in time
    /// @param now_s Current real time
    void updateDtS(const ros::Time& now_s) noexcept;

    /// @brief Helper function to update the error
    void updateError() noexcept;

    /// @brief Helpers to calculate various components of the controller
    /// @return Component value
    /// @{
    float64_t calculateP() const noexcept;
    float64_t calculateI()       noexcept;
    float64_t calculateD() const noexcept;
    /// @}

    /// @brief ctl filter helper function
    /// @param input The input to filter into output
    float64_t filterOutput(const float64_t input);


    float64_t m_setpoint{0.0}; ///< Setpoint
    float64_t m_feedback{0.0}; ///< Feedback
    float64_t m_output{0.0};   ///< Output

    ros::Duration m_dt_s;          ///< Current delta time, in seconds
    ros::Time     m_last_time_s;   ///< Last timestamp, used to track deltas

    float64_t m_error{0.0};         ///< Current error
    float64_t m_integral{0.0};      ///< Integral
    float64_t m_last_error{0.0};    ///< Last error (previous iteration)

    boost::circular_buffer<float64_t> m_ctl_buffer; ///< Circular buffer for ctl filter

    PIDConfig m_cfg;           ///< PID Config
};

} // namespace vi

#endif // VI_PID_controller_HPP
