#ifndef _SERVO_HANDLER_HPP
#define _SERVO_HANDLER_HPP


namespace ServoHandler
{
    void init();
    // void servoWrite(float angle);
    // void servoUp();
    // void servoDown();

    void armPositionUp();
    void armPositionDown();
    void armPositionFold();
    void pumpOn();
    void pumpOff();
}

#endif