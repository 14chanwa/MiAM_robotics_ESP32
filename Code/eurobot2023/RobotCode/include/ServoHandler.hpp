#ifndef _SERVO_HANDLER_HPP
#define _SERVO_HANDLER_HPP


namespace ServoHandler
{
    void init();
    void servoWrite(float angle);
    void servoUp();
    void servoDown();
}

#endif