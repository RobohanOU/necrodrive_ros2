/****************************NATSUROBOCON ROBOHAN*****************************
    🮕             🮘  🭦🮄🮄🮄🮄🮃🮃🮃🮃🮃🮃🮂🮂🮂🮂🭛   🮘 🭦🭏▂▂▂▂▃▃▃█🭍▄▅▅▆▆   🭔🭀  🮘       🮕
         🮘     🮕    🮕 ⎞ 🭦🮄🮄🮄║🮃🮃🮂🮂🮂 ⎛🮘           🭅🭛  🭥🭔🭍🭑▂▁▂🭐 🭭     🮕
             🮘    🮕   ║ 🭦🮄🮄🮃║🮃🮃🮂🮂🭛 ║  🮕   🮘 ║🮀🮀🮀🮀🮀🮀🮀🮀🮀🮀▌ 🮂🮂   🮕              🮕
        🮕          🮘 🭵🭱  ║  ║   ║ 🭵🭱        ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬍          🮘 
             🮕       🭴🭰 🭋▂▂▃║▃▄▄🭛  ║  🮘     ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋   🮕        🮕
      🮕        🮘   🮕 ║     🭵🭱      🭴🭰    🮕  🭦🮄🮄🮄🮄🮄🮃🮃🮃🮃🮂🮂🮂🮂🮂🭌  🮘     🮕
                  🮘  ║     ║▁▂▂▃🭎   ║🭋🭡 🮘  🭅🭀 🭃🭌  🭃🭌  🭃🭌   ▊   
        🮕      🮕    ⎠⎠ 🭦🮅🮅🮄🮄🮂🮂   🭦   V     🭒🭡 🭦🭡  🭦🭡  🭦🭡 🭦🭩🭡     🮘    🮕   
****************************necrodrive_system.cpp****************************/ 
/**
 * Maintainer: Oz
 * Description: Necrodrive ros2_control hardware interface. See header for more detail
 * Robohan 2025
 */

#include "necrodrive_ros2/necrodrive_system.hpp"

namespace necrodrive_system
{

namespace hwi = hardware_interface;
constexpr ros2socketcan::FrameType DATAFRAME = ros2socketcan::FrameType::DATA;
constexpr ros2socketcan::FrameType RTRFRAME = ros2socketcan::FrameType::REMOTE;


//-------------------------------------UTILITIES-------------------------------------------

/**
 * \brief   applies command to node id
 * \returns combined id and command
 */
auto NecrodriveSystem::get_can_id_(uint8_t command, bool is_rtr) -> ros2socketcan::CanId
{
    uint8_t idcmd = odrv_id_ << 5 | command;
    //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "CMDID: 0x%X", idcmd);
    return ros2socketcan::CanId { idcmd, 0, (is_rtr) ? RTRFRAME : DATAFRAME, ros2socketcan::StandardFrame};
}


//----------------------------------HARDWARE INTERFACE-------------------------------------

auto NecrodriveSystem::on_init(const hwi::HardwareInfo& info) -> hwi::CallbackReturn
{
    if (hwi::SystemInterface::on_init(info) != hwi::CallbackReturn::SUCCESS)
    {
        return hwi::CallbackReturn::ERROR;
    }

    // get data from xacro
    interface_ = info.hardware_parameters.at("can_interface");
    odrv_id_ = std::stoi(info.hardware_parameters.at("odrive_id"));
    write_timeout_ns_ = std::chrono::nanoseconds(
        std::stoi(info.hardware_parameters.at("write_timeout_ns"))
    );
    read_timeout_ns_ = std::chrono::nanoseconds(
        std::stoi(info.hardware_parameters.at("read_timeout_ns"))
    );
    heartbeat_timeout_ms_ = std::chrono::milliseconds(
        std::stoi(info.hardware_parameters.at("heartbeat_timeout_ms"))
    );

    RCLCPP_INFO(get_logger(), "target interface: %s", interface_.c_str());

    // check setup
    for (const hwi::ComponentInfo& joint : info_.joints)
    {
        // command interface should include position or vel
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(get_logger(), 
                "Joint '%s' has %zu command interfaces. It should have 1 (pos or vel)", 
                joint.name.c_str(), joint.command_interfaces.size()
            );
            return hwi::CallbackReturn::ERROR;
        }

        // command interface type check
        if (joint.command_interfaces[0].name == hwi::HW_IF_POSITION)
        {
            mode_ = POSITION_CONTROL;
            RCLCPP_INFO(get_logger(), "Control mode set to position");
        }
        else if (joint.command_interfaces[0].name == hwi::HW_IF_VELOCITY)
        {
            mode_ = VELOCITY_CONTROL;
            RCLCPP_INFO(get_logger(), "Control mode set to velocity");
        }
        else
        {   
            RCLCPP_FATAL(get_logger(), 
                "Joint '%s' has command inferace %s. Expected %s or %s",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(), 
                hwi::HW_IF_POSITION, hwi::HW_IF_VELOCITY
            );
            return hwi::CallbackReturn::ERROR;
        }

        // state interface check
        // position and velocity - 2
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has %zu state interfaces. 2 expected", 
                joint.name.c_str(), joint.state_interfaces.size()
            );
            return hwi::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hwi::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hwi::HW_IF_POSITION);
            return hwi::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hwi::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hwi::HW_IF_VELOCITY);
            return hwi::CallbackReturn::ERROR;
        }

    }
    
    // everything OK!
    return hwi::CallbackReturn::SUCCESS;
}

auto NecrodriveSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    -> hwi::CallbackReturn
{
    RCLCPP_INFO(get_logger(), "Configurating necrodrive..");

    // allocate sender
    if (!pSender_)
    {
        try 
        {
            pSender_ = std::make_unique<ros2socketcan::SocketCanSender>(interface_, false);
        } catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), 
                "Couldn't create a SocketCanSender object at %s!: %s", 
                interface_.c_str(), e.what()
            );
            return hwi::CallbackReturn::ERROR;
        }
    }


    // allocate receiver
    if (!pCanReader_)
    {
        try
        {
            auto pReceiver = std::make_shared<ros2socketcan::SocketCanReceiver>(interface_, false);
            pCanReader_ = std::make_unique<CanReader>(
                get_logger(),
                get_clock(),
                std::make_shared<ros2socketcan::SocketCanSender>(interface_, false),
                pReceiver,
                write_timeout_ns_,
                read_timeout_ns_,
                heartbeat_timeout_ms_
            );
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), 
                "Couldn't create a CanReader object at %s!: %s", 
                interface_.c_str(), e.what()
            );
        }
    }

    // reset values
    for (const auto& [name, descr] : joint_state_interfaces_)
    {
        set_state(name, 0.0);
    }
    for (const auto& [name, descr] : joint_command_interfaces_)
    {
        set_command(name, 0.0);
    }

    // try to get line voltage
    pCanReader_->start();

    auto request_future = pCanReader_->send_request<bus_voltage_current_t>(
        std::make_shared<CanRequest<bus_voltage_current_t>>(get_can_id_(odrive_cmds::GET_BUS_VOLTAGE_CURRENT, true))
    );

    auto voltage_current = request_future.get();

    RCLCPP_INFO(get_logger(), "Configuration complete! Bus voltage: %f", voltage_current.voltage);

    pCanReader_->stop();
    return hwi::CallbackReturn::SUCCESS;
}

auto NecrodriveSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    -> hwi::CallbackReturn
{
    RCLCPP_INFO(get_logger(), "Activating necrodrive..");
    pCanReader_->start();
    // set control mode to closed loop control
    send_odrive_<uint32_t>(odrive_cmds::AXIS_REQUESTED_STATE, axis_state::CLOSED_LOOP_CONTROL);
    // set input mode appropriately
    controller_mode_t controller_mode;
    if (mode_ == POSITION_CONTROL)
    {
        controller_mode.control_mode = control_mode::POSITION_CONTROL;
        controller_mode.input_mode = input_mode::TRAP_TRAJ;
        RCLCPP_INFO(get_logger(), "Setting control mode to POSITION CONTROL and input mode to TRAP TRAJ");
    } else {
        controller_mode.control_mode = control_mode::VELOCITY_CONTROL;
        controller_mode.input_mode = input_mode::VEL_RAMP;
    }
    send_odrive_<controller_mode_t>(odrive_cmds::SET_CONTROLLER_MODES, controller_mode);

    RCLCPP_INFO(get_logger(), "Activation complete!");

    return hwi::CallbackReturn::SUCCESS;
}

auto NecrodriveSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    -> hwi::CallbackReturn
{
    RCLCPP_INFO(get_logger(), "Deactivating necrodrive...");

    send_odrive_<uint32_t>(odrive_cmds::AXIS_REQUESTED_STATE, axis_state::IDLE);
    pCanReader_->stop();
    
    RCLCPP_INFO(get_logger(), "Deactivating completed!");

    return hwi::CallbackReturn::SUCCESS;
}

auto NecrodriveSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    -> hwi::return_type
{
    // get pos and vel
    auto estimate_future = pCanReader_->send_request<encoder_estimates_t>(
        std::make_shared<CanRequest<encoder_estimates_t>>(get_can_id_(odrive_cmds::GET_ENCODER_ESTIMATES, true))
    );
    auto estimates = estimate_future.get();

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION) // pos
        {   
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "read pos: %f vel: %f", estimates.pos, estimates.vel);
            set_state(name, static_cast<double>(estimates.pos));
        } else { // vel
            set_state(name, static_cast<double>(estimates.vel));
        }
    }

    return hwi::return_type::OK;
}

auto NecrodriveSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    -> hwi::return_type
{
    for (const auto& [name, descr] : joint_command_interfaces_)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "write: %f", get_command(name));
        // read from command interface
        if (mode_ == POSITION_CONTROL)
        {
            input_pos_t inputs;
            inputs.input_pos = get_command(name);
            send_odrive_(odrive_cmds::SET_INPUT_POS, inputs);
        } else { // velocity control
            input_vel_t inputs;
            inputs.input_vel = get_command(name);
            send_odrive_(odrive_cmds::SET_INPUT_VEL, inputs);
        }
    }
    
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(necrodrive_system::NecrodriveSystem, hardware_interface::SystemInterface)