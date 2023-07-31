#include <Motor.hpp>
#include <tasks.hpp>

Motor::Motor(
    uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2,
    unsigned char encoderID)
{
    motorDriver = new L298N(
        pinEnable, 
        pinIN1, 
        pinIN2
    );
    encoderID_ = encoderID;

    motorPID = new miam::PID(VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 1000);
}

void Motor::setSpeed(int speed_rpm)
{
    targetSpeed_rpm = speed_rpm;
}
double Motor::getSpeed()
{
    return currentSpeed_rpm;
}

void Motor::tickControl()
{
    currentTime = micros();
    if (timeLowLevel > 0)
    {
        dt_ms = (currentTime - timeLowLevel) / 1000.0; // in ms
       
        // speed in rpm
        currentSpeed_rpm = get_encoder_speed(encoderID_) 
            * 60.0 / 0.01 // convert from 10ms to 60s
            / ENCODER_PULSE_PER_REVOLUTION // convert from encoder tick to encoder revolution
            / MOTOR_REDUCTION_FACTOR; // convert from encoder revolution to motor revolution

        error = currentSpeed_rpm - targetSpeed_rpm; // in rpm

        correction = motorPID->computeValue(error, dt_ms);
        newTarget = targetSpeed_rpm + correction;

        // convert from rpm to 0-255
        basePWMTarget = (targetSpeed_rpm < 0 ? -1.0 : 1.0) 
            * std::min((int)(std::abs(targetSpeed_rpm * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);
        newPWMTarget = (newTarget < 0 ? -1.0 : 1.0) 
            * std::min((int)(std::abs(newTarget * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);
        // newPWMTarget = 100;

        if (newPWMTarget >= 0)
        {
            motorDriver->forward();
        }
        else
        {
            motorDriver->backward();
        }
        motorDriver->setSpeed(std::abs(newPWMTarget));

        // if (abs(targetSpeed_rpm) < 10)
        // {
        //     motorPID->resetIntegral();
        // }
    }
    timeLowLevel = currentTime;
}

void Motor::printToSerial()
{
    Serial.print(">currentSpeed:");
    Serial.println(currentSpeed_rpm);
    Serial.print(">targetSpeed: ");
    Serial.println(targetSpeed_rpm);  
    Serial.print(">error: ");
    Serial.println(error);       
    Serial.print(">correction:");
    Serial.println(correction);        
    // Serial.print(" - currentSpeed: ");
    // Serial.print(currentSpeed);             
    // Serial.print(" - targetSpeed: ");
    // Serial.print(targetSpeed);           
    // Serial.print(" - correction: ");
    // Serial.print(correction);           
    // Serial.print(" - dt_min: ");
    // Serial.print(dt_min);
    // Serial.print(" - error: ");
    // Serial.print(error);
    Serial.print(">basePWMTarget:");
    Serial.println(basePWMTarget);
    Serial.print(">newPWMTarget:");
    Serial.println(newPWMTarget);
    // Serial.print(" - newPWMTarget: ");
    // Serial.println(newPWMTarget);
}