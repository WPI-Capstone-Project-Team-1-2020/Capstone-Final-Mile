#ifndef VI_VEHICLE_INTERFACE_HPP
#define VI_VEHICLE_INTERFACE_HPP

// Ros
#include <ros/ros.h>

// Standard
#include <memory>

namespace vi
{

// Forward Declarations
class VehicleInterfaceConfig;
class Controller;
class TopicPublisher;
class TopicSubscriber;
class TrajectoryUnspooler;

/// @brief Class for vehicle interface to publish commands to vehicle
class VehicleInterface
{
public:
    /// @brief Default Constructor
    /// @param nh nodehandle
    /// @param pnh private nodehandle
    VehicleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /// @brief Default destructor    
    ~VehicleInterface();

private:
    /// @brief Main driving function of the vehicle interface
    /// @param event Timer event
    void update(const ros::TimerEvent& event);

    /// @brief Updates diagnostics
    /// @param health `true` if healthy
    void updateDiagnostics(const bool health);
    
    std::shared_ptr<VehicleInterfaceConfig> m_cfg;         ///< Configuration of vehicle interface
    std::unique_ptr<Controller>             m_controller;  ///< Controller
    std::unique_ptr<TopicSubscriber>        m_topic_sub;   ///< Topic Subscriber
    std::unique_ptr<TopicPublisher>         m_topic_pub;   ///< Topic Publisher
    std::unique_ptr<TrajectoryUnspooler>    m_unspooler;   ///< Trajectory Unspooler
    ros::Timer                              m_timer;       ///< Timer to drive update cycles
};

} // namespace vi

#endif // VI_VEHICLE_INTERFACE_HPP