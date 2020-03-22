#ifndef VI_TRAJECTORY_UNSPOOLER_HPP
#define VI_TRAJECTORY_UNSPOOLER_HPP

// Component
#include "VehicleInterfaceData.hpp"

// Libraries
#include <boost/cstdfloat.hpp>

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
class TrajectoryUnspooler
{
public:
   /// @brief Default constructor
   TrajectoryUnspooler() = default;

   /// @brief Destructor for forward declares
   ~TrajectoryUnspooler();

   /// @brief Main driving function
   /// @param now_s Current program time
   void update(const ros::Time& now_s);

   /// @brief Accessor for command
   /// @return Command
   const geometry_msgs::Twist& getCommand() const noexcept {return m_cmd;}

   /// @brief Mutator for VI data
   /// @param val Val
   /// @{
   void setVehicleInterfaceData(const VehicleInterfaceData& val) noexcept {m_data = val;}
   void setVehicleInterfaceData(VehicleInterfaceData&& val)      noexcept {m_data = val;}
   /// @}


private:
    /// @brief Calculates a command to be send based on trajectory, time, and pose
    /// @param now_s Time at which update was called
    void calculateCommandFromTrajectory(const ros::Time& now_s);


    VehicleInterfaceData m_data; ///< Vehicle Interface Data
    geometry_msgs::Twist m_cmd;  ///< Output command
};

} // namespace vi

#endif // VI_TRAJECTORY_UNSPOOLER_HPP
