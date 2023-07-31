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
            uint8_t pinEncoderA, uint8_t pinEncoderB);
        
        // set target speed in rad/s
        void setWheelSpeed(double speed);
        
         // get current in rad/s
        double getWheelSpeed();
        double getWheelPosition(); // in rad

        void startLowLevelLoop();

        // functions used to setup interrupt
        const uint8_t pinEncoderA_;
        const uint8_t pinEncoderB_;
        void handleEncoderInterrupt();

    private:

        // functions that will be run in tasks have to be static
        static void tickMotorControl(void* parameters);
        static void tickEncoderSpeed(void* parameters);
        static void tickPrintToSerial(void* parameters);

        miam::PID* motorPID;
        L298N* motorDriver;

        // variables for interrupt
        bool currentA;
        bool oldB;

        // encoder value in ticks
        volatile int encoder_value;
        int old_encoder_value;
        // encoder speed in ticks
        volatile int encoder_speed;

        // wheel speed in rad/s
        double targetSpeed;
        double currentSpeed;

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