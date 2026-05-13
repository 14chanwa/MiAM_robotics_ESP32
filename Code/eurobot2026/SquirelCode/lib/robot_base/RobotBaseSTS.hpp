#ifndef _ROBOTBASESTS_HPP
#define _ROBOTBASESTS_HPP

#include <Arduino.h>
#include "AbstractRobotBase.hpp"

class RobotWheelSTS : public AbstractRobotWheel
{
    public:
        RobotWheelSTS(byte servo_id, std::string prefix, bool inverted);

        // print variables to serial
        void printToSerial();

        // low level loop functions
        void updateMotorControl(bool motorEnabled);
        void updateEncoderSpeed();

        float getWheelSpeed();

        byte servo_id_;
        bool inverted_;

        // variables for loop
        float currentVelocity_;

        unsigned long timeLowLevel_;
        unsigned long currentTime_;
        float dt_ms_;
        float error_;
        float correction_;

        int baseTarget_;
        int newTarget_;

        float lastEncoderReading_ = 0.0;
        unsigned long oldTimeEncoderSpeed_ = 0.0;
};

class RobotBaseSTS : public AbstractRobotBase
{
    protected:
        RobotBaseSTS();

    public:
        void setup();
        DrivetrainMeasurements getMeasurements();
        RobotParameters getParameters();

        static AbstractRobotBase* getInstance();

        RobotWheelSTS* leftWheel_;
        RobotWheelSTS* rightWheel_;

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