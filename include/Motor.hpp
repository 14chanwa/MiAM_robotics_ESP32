#ifndef _MOTOR_HPP
#define _MOTOR_HPP

#include <Arduino.h>
#include <L298N.h>
#include <PID.h>

class Motor
{
    public:
        Motor(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, unsigned char encoderID);
        void setSpeed(int speed_rpm);
        double getSpeed();
        void tickControl();

        void printToSerial();

    private:
        miam::PID* motorPID;
        L298N* motorDriver;
        unsigned char encoderID_;

        double targetSpeed_rpm;
        double currentSpeed_rpm;

        double motorControl_PWM;

        // variables for loop
        unsigned long timeLowLevel;
        unsigned long currentTime;
        double dt_ms;
        double error;
        double correction;
        double newTarget;
        int basePWMTarget;
        int newPWMTarget;
};

#endif