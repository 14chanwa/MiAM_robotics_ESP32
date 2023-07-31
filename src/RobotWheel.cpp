#include <RobotWheel.hpp>
#include <parameters.hpp>

RobotWheel::RobotWheel(
    uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, 
            uint8_t pinEncoderA, uint8_t pinEncoderB) :
            pinEncoderA_(pinEncoderA), pinEncoderB_(pinEncoderB),
            // explicitely initialize volatile values
            encoder_value(0), encoder_speed(0)
{
    motorDriver = new L298N(
        pinEnable, 
        pinIN1, 
        pinIN2
    );
    motorPID = new miam::PID(VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 1000);
}

void RobotWheel::startLowLevelLoop()
{

    xTaskCreate(
        RobotWheel::tickEncoderSpeed, 
        "tickEncoderSpeed",
        1000,
        this,
        1,
        NULL
    ); 
    vTaskDelay(3.0 / portTICK_PERIOD_MS);

    xTaskCreate(
        RobotWheel::tickMotorControl, 
        "tickMotorControl",
        1000,
        this,
        1,
        NULL
    ); 
    vTaskDelay(3.0 / portTICK_PERIOD_MS);

    xTaskCreate(
        RobotWheel::tickPrintToSerial, 
        "tickPrintToSerial",
        1000,
        this,
        1,
        NULL
    ); 
    vTaskDelay(3.0 / portTICK_PERIOD_MS);
}

void RobotWheel::handleEncoderInterrupt()
{
    // Get current status.
    bool currentA = digitalRead(pinEncoderA_);

    // The direction of the encoder is given by currentA xor oldB
    encoder_value += (oldB ^ currentA ? 1 : -1);
    oldB = digitalRead(pinEncoderB_);
}

void RobotWheel::setWheelSpeed(int speed_rpm)
{
    targetSpeed_rpm = speed_rpm;
}
double RobotWheel::getWheelSpeed()
{
    return currentSpeed_rpm;
}

void RobotWheel::tickMotorControl(void* parameters)
{
    RobotWheel* robotWheel = static_cast<RobotWheel*>(parameters);

    for (;;)
    {
        robotWheel->currentTime = micros();
        if (robotWheel->timeLowLevel > 0)
        {
            robotWheel->dt_ms = (robotWheel->currentTime - robotWheel->timeLowLevel) / 1000.0; // in ms
        
            // speed in rpm
            robotWheel->currentSpeed_rpm = robotWheel->encoder_speed
                * 60.0 / 0.01 // convert from 10ms to 60s
                / ENCODER_PULSE_PER_REVOLUTION // convert from encoder tick to encoder revolution
                / MOTOR_REDUCTION_FACTOR; // convert from encoder revolution to motor revolution

            robotWheel->error = robotWheel->currentSpeed_rpm - robotWheel->targetSpeed_rpm; // in rpm

            robotWheel->correction = robotWheel->motorPID->computeValue(robotWheel->error, robotWheel->dt_ms);
            robotWheel->newTarget = robotWheel->targetSpeed_rpm + robotWheel->correction;

            // convert from rpm to 0-255
            robotWheel->basePWMTarget = (robotWheel->targetSpeed_rpm < 0 ? -1.0 : 1.0) 
                * std::min((int)(std::abs(robotWheel->targetSpeed_rpm * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);
            robotWheel->newPWMTarget = (robotWheel->newTarget < 0 ? -1.0 : 1.0) 
                * std::min((int)(std::abs(robotWheel->newTarget * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);
            // newPWMTarget = 100;

            if (robotWheel->newPWMTarget >= 0)
            {
                robotWheel->motorDriver->forward();
            }
            else
            {
                robotWheel->motorDriver->backward();
            }
            robotWheel->motorDriver->setSpeed(std::abs(robotWheel->newPWMTarget));
        }
        robotWheel->timeLowLevel = robotWheel->currentTime;
        vTaskDelay(10.0 / portTICK_PERIOD_MS);
    }    
}

void RobotWheel::tickEncoderSpeed(void* parameters)
{
    RobotWheel* robotWheel = static_cast<RobotWheel*>(parameters);
    for (;;)
    {
        robotWheel->encoder_speed = 
            robotWheel->encoder_value - robotWheel->old_encoder_value;
        robotWheel->old_encoder_value = robotWheel->encoder_value;
        vTaskDelay(10.0 / portTICK_PERIOD_MS);
    }    
}

void RobotWheel::tickPrintToSerial(void* parameters)
{
    RobotWheel* robotWheel = static_cast<RobotWheel*>(parameters);
    for (;;)
    {
        Serial.print(">encoder_value:");
        Serial.println(robotWheel->encoder_value);
        Serial.print(">encoder_speed:");
        Serial.println(robotWheel->encoder_speed);
        Serial.print(">currentSpeed:");
        Serial.println(robotWheel->currentSpeed_rpm);
        Serial.print(">targetSpeed: ");
        Serial.println(robotWheel->targetSpeed_rpm);  
        Serial.print(">error: ");
        Serial.println(robotWheel->error);       
        Serial.print(">correction:");
        Serial.println(robotWheel->correction);        
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
        Serial.println(robotWheel->basePWMTarget);
        Serial.print(">newPWMTarget:");
        Serial.println(robotWheel->newPWMTarget);
        // Serial.print(" - newPWMTarget: ");
        // Serial.println(newPWMTarget);
        vTaskDelay(100.0 / portTICK_PERIOD_MS);
    } 
}