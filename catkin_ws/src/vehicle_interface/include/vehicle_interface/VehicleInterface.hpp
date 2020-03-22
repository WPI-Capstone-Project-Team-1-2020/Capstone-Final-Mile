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
class TopicPublisher;
class TopicSubscriber;

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

    std::shared_ptr<VehicleInterfaceConfig> m_cfg;         ///< Configuration of vehicle interface
    std::unique_ptr<TopicSubscriber>        m_topic_sub;   ///< Topic Subscriber
    std::unique_ptr<TopicPublisher>         m_topic_pub;   ///< Topic Publisher
    ros::Timer                              m_timer;       ///< Timer to drive update cycles
};

} // namespace vi

#endif // VI_VEHICLE_INTERFACE_HPP