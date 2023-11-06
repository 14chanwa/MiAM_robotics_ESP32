#ifndef _ROBOTBASESTEPPER_HPP
#define _ROBOTBASESTEPPER_HPP

#include <Arduino.h>
#include "AbstractRobotBase.hpp"
#include <AccelStepper.h>

// class RobotWheelStepper : public AbstractRobotWheel
// {
//     public:
//         RobotWheelStepper(
//             uint8_t pinStep, uint8_t pinDir,
//             std::string prefix);

//         // print variables to serial
//         void printToSerial();

//         // low level loop functions
//         void updateMotorControl();
//         void updateEncoderSpeed();

//         AccelStepper* motorDriver;


//         // encoder value in ticks
//         volatile int encoderValue_;
//         int oldEncoderValue_;
//         unsigned long oldTimeEncoderSpeed_;

//         // variables for loop
//         unsigned long timeLowLevel_;
//         unsigned long currentTime_;
//         float dt_ms_;
//         float error_;
//         float correction_;

//         int baseTarget_;
//         int newTarget_;
// };

class RobotBaseStepper : public AbstractRobotBase
{
    protected:
        RobotBaseStepper();
    
    public:
        void setup();
        DrivetrainMeasurements getMeasurements();
        RobotParameters getParameters();
    
        static AbstractRobotBase* getInstance();

        // RobotWheelStepper* leftWheel_;
        // RobotWheelStepper* rightWheel_;

        virtual AbstractRobotWheel* getLeftWheel()
        {
            return NULL; //static_cast<AbstractRobotWheel* >(leftWheel_);
        };
        virtual AbstractRobotWheel* getRightWheel()
        {
            return NULL; //static_cast<AbstractRobotWheel* >(rightWheel_);
        };

        // Reimplements base speed to avoid powering motors when still
        void setBaseSpeed(DrivetrainTarget target);
        void updateControl();
        void updateSensors();

        // wheel high level 
        float targetSpeed_left_ = 0.0;
        volatile float currentSpeed_left_ = 0.0;
        float targetSpeed_right_ = 0.0;
        volatile float currentSpeed_right_ = 0.0;

        miam::PID* motorPID_left;
        miam::PID* motorPID_right;

        // encoder value in ticks
        volatile int encoderValue_left_ = 0;
        int oldEncoderValue_left_ = 0;
        volatile int encoderValue_right_ = 0;
        int oldEncoderValue_right_ = 0;
        unsigned long oldTimeEncoderSpeed_ = 0;

        // variables for loop
        unsigned long timeLowLevel_ = 0;
        unsigned long currentTime_ = 0;
        float dt_ms_ = 0;
        float error_left_ = 0;
        float error_right_ = 0;
        float correction_left_ = 0;
        float correction_right_ = 0;

        int baseTarget_left_ = 0;
        int newTarget_left_ = 0;
        int baseTarget_right_ = 0;
        int newTarget_right_ = 0;
};


#endif