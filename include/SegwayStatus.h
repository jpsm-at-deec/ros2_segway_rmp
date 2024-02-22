#ifndef _segway_rmp_SegwayStatus_h
#define _segway_rmp_SegwayStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

namespace segway_rmp
{

    class SegwayStatus 
    {
        public:
            float pitch_angle;
            float pitch_rate;
            float roll_angle;
            float roll_rate;
            float left_wheel_velocity;
            float right_wheel_velocity;
            float yaw_rate;
            float servo_frames;
            float left_wheel_displacement;
            float right_wheel_displacement;
            float forward_displacement;
            float yaw_displacement;
            float left_motor_torque;
            float right_motor_torque;
            int8_t operation_mode;
            int8_t gain_schedule;
            float ui_battery;
            float powerbase_battery;
            bool motors_enabled;
            enum { LIGHT = 1 };
            enum { TALL = 2 };
            enum { HEAVY = 3 };
            enum { BALANCE = 1 };
            enum { TRACTOR = 2 };
            enum { POWER_DOWN = 3 };

        SegwayStatus():
            pitch_angle(0),
            pitch_rate(0),
            roll_angle(0),
            roll_rate(0),
            left_wheel_velocity(0),
            right_wheel_velocity(0),
            yaw_rate(0),
            servo_frames(0),
            left_wheel_displacement(0),
            right_wheel_displacement(0),
            forward_displacement(0),
            yaw_displacement(0),
            left_motor_torque(0),
            right_motor_torque(0),
            operation_mode(0),
            gain_schedule(0),
            ui_battery(0),
            powerbase_battery(0),
            motors_enabled(0)
        {
        }
    };

}
#endif