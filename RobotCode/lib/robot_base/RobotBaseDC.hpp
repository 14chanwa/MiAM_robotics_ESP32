#ifndef _ROBOTBASEDC_HPP
#define _ROBOTBASEDC_HPP

#include <Arduino.h>
#include <L298N.h>
#include <PID.h>
#include "AbstractRobotBase.hpp"

/////////////////////////////////////////////
// Pinout
/////////////////////////////////////////////

#define IN1_A 16
#define IN2_A 17
#define IN1_B 32
#define IN2_B 33
#define EN_A 27
#define EN_B 14

#define ENCODER_A1 13
#define ENCODER_B1 4

#define ENCODER_A2 34
#define ENCODER_B2 35

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

#define WHEEL_RADIUS_MM 30.0f
#define WHEEL_SPACING_MM 40.0f

// conversion functions
#define RPM_TO_RAD_S(VALUE) (VALUE * 2.0f * M_PI / 60.0f)
#define RAD_S_TO_RPM(VALUE) (VALUE * 60.0f / (2.0f * M_PI))

// give 20% overhead
#define MAX_SPEED_RPM (MOTOR_RATED_RPM)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 250.0f

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_REDUCTION_FACTOR 100.0f
#define MOTOR_RATED_RPM 220.0f
#define MOTOR_CONTROL_FREQUENCY_HZ 10000

#define ENCODER_PULSE_PER_REVOLUTION 28.0f
#define ENCODER_SPEED_TICK_PERIOD_MS 10.0f

/////////////////////////////////////////////
// Wheel PID
/////////////////////////////////////////////

// Target control will be
// PWM = targetSpeed (rpm) * 255 / max speed (rpm) * MOTOR_TARGET_CONTROL_A + MOTOR_TARGET_CONTROL_B
#define MOTOR_TARGET_CONTROL_B 20
#define MOTOR_TARGET_CONTROL_A 1.0f
// offset will be applied if targetSpeed (rad/s) is above threshold
#define MOTOR_ST0P_THRESHOLD_RAD_S 0.02f

// Wheel PID parameters
#define VELOCITY_KP 0.2f
#define VELOCITY_KD 0.0f
#define VELOCITY_KI 0.05f

class RobotWheelDC : public AbstractRobotWheel
{
    public:
        RobotWheelDC(
            uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, 
            uint8_t pinEncoderA, uint8_t pinEncoderB,
            std::string prefix, uint8_t pwmChannel);
        
        // set target speed in rad/s
        void setWheelSpeed(float speed);
        
         // get current in rad/s
        float getWheelSpeed(); // in rad/s

        // functions used to setup interrupt
        const uint8_t pinEncoderA_;
        const uint8_t pinEncoderB_;
        void handleEncoderInterrupt();

        // print variables to serial
        void printToSerial();

        // low level loop functions
        void updateMotorControl();
        void updateEncoderSpeed();

        miam::PID* motorPID;
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
        void setBaseSpeed(DrivetrainTarget target);
        void updateSensors();
        void updateControl();
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
};


#endif