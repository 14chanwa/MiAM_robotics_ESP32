#ifdef USE_STEPPER_MOTORS
#ifndef _ROBOTBASESTEPPER_HPP
#define _ROBOTBASESTEPPER_HPP

#include <Arduino.h>
#include "AbstractRobotBase.hpp"

class RobotBaseStepper : public AbstractRobotBase
{
    protected:
        RobotBaseStepper();
    
    public:
        void setup();
        DrivetrainMeasurements getMeasurements();
        RobotParameters getParameters();
    
        static AbstractRobotBase* getInstance();

        virtual AbstractRobotWheel* getLeftWheel()
        {
            return NULL; //static_cast<AbstractRobotWheel* >(leftWheel_);
        };
        virtual AbstractRobotWheel* getRightWheel()
        {
            return NULL; //static_cast<AbstractRobotWheel* >(rightWheel_);
        };

        void forceStop();

        // Reimplements base speed to avoid powering motors when still
        void setBaseSpeed(DrivetrainTarget target);
        void updateControl(bool motorEnabled);
        void updateSensors();

        void setBlockWheels(bool blockWheels);

        // unsigned long getStepIntervalRight();
        // unsigned long getStepIntervalLeft();

        // wheel high level 
        float targetSpeed_left_ = 0.0;
        volatile float currentSpeed_left_ = 0.0;
        float targetSpeedDriver_left_ = 0.0;

        float targetSpeed_right_ = 0.0;
        volatile float currentSpeed_right_ = 0.0;
        float targetSpeedDriver_right_ = 0.0;

        // miam::PID* motorPID_left;
        // miam::PID* motorPID_right;

        int8_t leftStepperResult_ = 0;
        int8_t rightStepperResult_ = 0;

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

        bool isRunningLeft = false;
        bool isRunningRight = false;
        bool isEmptyLeft = false;
        bool isEmptyRight = false;
        bool isGenActiveLeft = false;
        bool isGenActiveRight = false;
        bool isFullLeft = false;
        bool isFullRight = false;

        bool desyncDetectedLeft = false;
        bool desyncDetectedRight = false;
};


#endif
#endif