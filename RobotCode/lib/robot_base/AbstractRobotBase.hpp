#ifndef _ABSTRACTROBOTBASE_HPP
#define _ABSTRACTROBOTBASE_HPP

#include <string>
#include <DrivetrainKinematics.h>
#include <RobotParameters.hpp>
#include <PID.h>


namespace side{
    int const RIGHT = 0;
    int const LEFT = 1;
}

typedef struct {
    Vector2 motorSpeed = Vector2::Zero(); ///<< Target motor speed, in rad/s
} DrivetrainTarget;

typedef struct {
    Vector2 motorSpeed; ///<< Measured motor speed, in rad/s
    // std::deque<DetectedRobot> lidarDetection; ///< Robots detected by the lidar.
} DrivetrainMeasurements;


class AbstractRobotWheel
{
    public:
        AbstractRobotWheel(std::string prefix);
        
        // set target speed in rad/s
        void setWheelSpeed(float speed);
        
         // get current in rad/s
        float getWheelSpeed(); // in rad/s

        // print variables to serial
        virtual void printToSerial() = 0;

        // low level loop functions
        virtual void updateMotorControl() = 0;
        virtual void updateEncoderSpeed() = 0;

        void printPrefix(const char* value_name);
        std::string prefix_;

        // wheel speed in rad/s
        float targetSpeed_ = 0.0;
        volatile float currentSpeed_ = 0.0;

        miam::PID* motorPID;
    
};

class AbstractRobotBase
{
    public:
        AbstractRobotBase() {

        };
        virtual void setup() = 0;
        virtual void setBaseSpeed(DrivetrainTarget target);
        virtual void updateControl();
        virtual void updateSensors();
        virtual DrivetrainMeasurements getMeasurements() = 0;
        virtual RobotParameters getParameters() = 0;
    
        // virtual static AbstractRobotBase* getInstance() = 0;

        virtual AbstractRobotWheel* getLeftWheel() = 0;
        virtual AbstractRobotWheel* getRightWheel() = 0;
};

#endif