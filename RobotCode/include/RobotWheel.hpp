#ifndef _ROBOTWHEEL_HPP
#define _ROBOTWHEEL_HPP

#include <Arduino.h>
#include <L298N.h>
#include <PID.h>

class RobotWheel
{
    public:
        RobotWheel(
            uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, 
            uint8_t pinEncoderA, uint8_t pinEncoderB,
            std::string prefix);
        
        // set target speed in rad/s
        void setWheelSpeed(double speed);
        
         // get current in rad/s
        double getWheelSpeed();
        double getWheelPosition(); // in rad

        // functions used to setup interrupt
        const uint8_t pinEncoderA_;
        const uint8_t pinEncoderB_;
        void handleEncoderInterrupt();

        // print variables to serial
        void printToSerial();

        // low level loop functions
        void updateMotorControl();
        void updateEncoderSpeed();

    private:

        void printPrefix(const char* value_name);
        std::string prefix_;

        miam::PID* motorPID;
        L298N* motorDriver;

        // variables for interrupt
        bool currentA;
        bool oldB;

        // encoder value in ticks
        volatile int encoderValue_;
        int oldEncoderValue_;
        unsigned long oldTimeEncoderSpeed_;

        // wheel speed in rad/s
        double targetSpeed_;
        volatile double currentSpeed_;

        // variables for loop
        unsigned long timeLowLevel_;
        unsigned long currentTime_;
        double dt_ms_;
        double error_;
        double PWMcorrection_;

        int basePWMTarget_;
        int newPWMTarget_;
};

#endif