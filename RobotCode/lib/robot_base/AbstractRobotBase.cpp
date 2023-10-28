#include <Arduino.h>
#include <AbstractRobotBase.hpp>

AbstractRobotWheel::AbstractRobotWheel(std::string prefix) : prefix_(prefix)
{

}

void AbstractRobotWheel::printPrefix(const char* value_name)
{
    Serial.print(">");
    Serial.print(prefix_.c_str());
    Serial.print(value_name);
    Serial.print(":");
}

void AbstractRobotWheel::setWheelSpeed(float speed)
{
    targetSpeed_ = speed;

    // // reset PID integral if target speed is zero to stop robot
    // if (std::abs(speed) < 1e-3)
    // {
    //     motorPID->resetIntegral();
    // }
}

float AbstractRobotWheel::getWheelSpeed()
{
    return currentSpeed_;
}

void AbstractRobotBase::setBaseSpeed(DrivetrainTarget target)
{
    getLeftWheel()->setWheelSpeed(target.motorSpeed[side::LEFT]);
    getRightWheel()->setWheelSpeed(target.motorSpeed[side::RIGHT]);
}

void AbstractRobotBase::updateControl() 
{
    getLeftWheel()->updateMotorControl();
    getRightWheel()->updateMotorControl();
}

void AbstractRobotBase::updateSensors() 
{
    getLeftWheel()->updateEncoderSpeed();
    getRightWheel()->updateEncoderSpeed();
}