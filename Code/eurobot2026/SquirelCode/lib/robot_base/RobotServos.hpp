#ifndef _SERVO_HANDLER_H
#define _SERVO_HANDLER_H

#include <Arduino.h>

namespace RobotServos
{
    void init();
    void init_servo_id_velocity(byte servo_id);
    void init_servo_id_position(byte servo_id);
    void set_servo_velocity(byte servo_id, int target_velocity);
    void set_servo_position(byte servo_id, int target_position);
    int get_current_speed(byte servo_id);
}

#endif