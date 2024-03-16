#ifndef _SERVO_HPP
#define _SERVO_HPP


namespace Servo
{
    void init();
    void servoWrite(float angle);
    void servoUp();
    void servoDown();
}

#endif