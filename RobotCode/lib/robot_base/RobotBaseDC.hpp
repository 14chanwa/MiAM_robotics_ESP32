#ifdef USE_DC_MOTORS
#ifndef _ROBOTBASEDC_HPP
#define _ROBOTBASEDC_HPP

#include <Arduino.h>
#include <L298N.h>
#include "AbstractRobotBase.hpp"

class RobotWheelDC : public AbstractRobotWheel
{
    public:
        RobotWheelDC(
            uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, 
            uint8_t pinEncoderA, uint8_t pinEncoderB,
            std::string prefix, uint8_t pwmChannel);

        // functions used to setup interrupt
        const uint8_t pinEncoderA_;
        const uint8_t pinEncoderB_;
        void handleEncoderInterrupt();

        // print variables to serial
        void printToSerial();

        // low level loop functions
        void updateMotorControl();
        void updateEncoderSpeed();

        L298N* motorDriver;

        // variables for interrupt
        bool currentA;
        bool oldB;

        // encoder value in ticks
        volatile int encoderValue_;
        int oldEncoderValue_;
        unsigned long oldTimeEncoderSpeed_;

        // variables for loop
        unsigned long timeLowLevel_;
        unsigned long currentTime_;
        float dt_ms_;
        float error_;
        float PWMcorrection_;

        int basePWMTarget_;
        int newPWMTarget_;
};

class RobotBaseDC : public AbstractRobotBase
{
    protected:
        RobotBaseDC();
    
    public:
        void setup();
        DrivetrainMeasurements getMeasurements();
        RobotParameters getParameters();
    
        static AbstractRobotBase* getInstance();

        RobotWheelDC* leftWheel_;
        RobotWheelDC* rightWheel_;

        virtual AbstractRobotWheel* getLeftWheel()
        {
            return static_cast<AbstractRobotWheel* >(leftWheel_);
        };
        virtual AbstractRobotWheel* getRightWheel()
        {
            return static_cast<AbstractRobotWheel* >(rightWheel_);
        };

        void forceStop() {};
};


#endif
#endif