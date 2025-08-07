#pragma once

#include <memory>

namespace necrodrive_system
{
namespace odrive_cmds
{
    const uint8_t ODRV_HEARTBEAT = 0x001;
    const uint8_t AXIS_REQUESTED_STATE = 0x007;
    const uint8_t GET_ENCODER_ESTIMATES = 0x009;
    const uint8_t SET_CONTROLLER_MODES = 0x00B;
    const uint8_t SET_INPUT_POS = 0x00C;
    const uint8_t SET_INPUT_VEL = 0x00D;
    const uint8_t GET_BUS_VOLTAGE_CURRENT = 0x017;
}

namespace axis_state
{
    const uint32_t IDLE = 0x1;
    const uint32_t CLOSED_LOOP_CONTROL = 0x8;
}

namespace control_mode
{
    const uint32_t VELOCITY_CONTROL = 0x2;
    const uint32_t POSITION_CONTROL = 0x3;
}

namespace input_mode
{
    const uint32_t PASSTHROUGH = 0x1;
    const uint32_t VEL_RAMP = 0x2;
    const uint32_t TRAP_TRAJ = 0x5;
}

struct controller_mode_t
{
    uint32_t control_mode = control_mode::POSITION_CONTROL;
    uint32_t input_mode = input_mode::TRAP_TRAJ;
};

struct bus_voltage_current_t 
{
    float voltage;
    float current;
};

struct encoder_estimates_t
{
    float pos;
    float vel;
};

struct input_pos_t
{
    float input_pos;
    int16_t vel_ff = 0;
    int16_t torque_ff = 0;
};

struct input_vel_t
{
    float input_vel = 0.0f;
    float torque_ff = 0.0f;
};

}