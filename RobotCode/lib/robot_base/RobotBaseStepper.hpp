#ifndef _ROBOTBASESTEPPER_HPP
#define _ROBOTBASESTEPPER_HPP

#include <Arduino.h>
#include "AbstractRobotBase.hpp"
#include <AccelStepper.h>

class RobotWheelStepper : public AbstractRobotWheel
{
    public:
        RobotWheelStepper(
            uint8_t pinStep, uint8_t pinDir,
            std::string prefix);

        // print variables to serial
        void printToSerial();

        // low level loop functions
        void updateMotorControl();
        void updateEncoderSpeed();

        AccelStepper* motorDriver;


        // encoder value in ticks
        volatile int encoderValue_;
        int oldEncoderValue_;
        unsigned long oldTimeEncoderSpeed_;

        // variables for loop
        unsigned long timeLowLevel_;
        unsigned long currentTime_;
        float dt_ms_;
        float error_;
        float correction_;

        int baseTarget_;
        int newTarget_;
};

class RobotBaseStepper : public AbstractRobotBase
{
    protected:
        RobotBaseStepper();
    
    public:
        void setup();
        DrivetrainMeasurements getMeasurements();
        RobotParameters getParameters();
    
        static AbstractRobotBase* getInstance();

        RobotWheelStepper* leftWheel_;
        RobotWheelStepper* rightWheel_;

        virtual AbstractRobotWheel* getLeftWheel()
        {
            return static_cast<AbstractRobotWheel* >(leftWheel_);
        };
        virtual AbstractRobotWheel* getRightWheel()
        {
            return static_cast<AbstractRobotWheel* >(rightWheel_);
        };

        // Reimplements base speed to avoid powering motors when still
        void setBaseSpeed(DrivetrainTarget target);
};


#endif