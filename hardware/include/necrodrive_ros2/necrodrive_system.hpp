#pragma once

#include <memory>
#include <string>
#include <cstddef>

// ros2
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ros2 control
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// socketcan
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

// odrive
#include "necrodrive_ros2/odrive_cmds.hpp"
#include "necrodrive_ros2/can_reader.hpp"

namespace necrodrive_system
{

namespace ros2socketcan = drivers::socketcan;

enum ControlMode
{
    POSITION_CONTROL,
    VELOCITY_CONTROL
};



class NecrodriveSystem : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(NecrodriveSystem)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    ros2socketcan::CanId get_can_id_(uint8_t command, bool is_rtr);
    
    template <typename T>
    void send_odrive_(uint8_t command, T data, uint8_t startBit=0);
    template <typename T>
    T getOdrive_(uint8_t command);

    
    std::string interface_;                             // name of the CAN socket interface
    uint32_t odrv_id_;                                  // id of the axis node
    std::chrono::nanoseconds write_timeout_ns_;         // timeout for sending messages
    std::chrono::nanoseconds read_timeout_ns_;          // timeout for reading messages
    std::chrono::milliseconds heartbeat_timeout_ms_;    // timeout for heartbeat

    ControlMode mode_ = POSITION_CONTROL;

    std::unique_ptr<drivers::socketcan::SocketCanSender> pSender_;
    std::unique_ptr<CanReader> pCanReader_;
};

}