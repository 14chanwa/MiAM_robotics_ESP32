#include <RobotWheel.hpp>
#include <parameters.hpp>
#include <cmath>

int rad_s_to_pwm_value(double speed_rad_s)
{
    // max pwm speed is supposed to be 255 for MAX_SPEED_RPM
    return(
        (speed_rad_s < 0 ? -1.0 : 1.0) // sign
            * std::min(
                (int)(
                    std::abs(RAD_S_TO_RPM(speed_rad_s)) 
                        * 255.0 / MAX_SPEED_RPM // scale 255 to max RPM
                        / 1.5 // comes from experience (motor at 255 does not reach MAX_SPEED_RPM)
                    ), 
                255 // cap to 255
            )
    );
}

double encoder_ticks_to_rad_s(int encoder_ticks)
{
    return(
        encoder_ticks 
            / (0.001 * ENCODER_SPEED_TICK_PERIOD_MS) // convert from 10ms to 1s
            / ENCODER_PULSE_PER_REVOLUTION // convert from encoder tick to encoder revolution
            / MOTOR_REDUCTION_FACTOR // convert from encoder revolution to motor revolution
            * 2.0 * M_PI // convert from revolution to rad
    );
}

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
    motorPID = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
        0.5 * MAX_SPEED_RAD_S / VELOCITY_KP // max integral is 50% of the max velocity
    );
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

void RobotWheel::setWheelSpeed(double speed)
{
    targetSpeed = speed;
}
double RobotWheel::getWheelSpeed()
{
    return currentSpeed;
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
        
            // speed in rad/s
            robotWheel->currentSpeed = encoder_ticks_to_rad_s(robotWheel->encoder_speed);

            robotWheel->error = robotWheel->currentSpeed - robotWheel->targetSpeed; // in rad/s

            robotWheel->correction = robotWheel->motorPID->computeValue(robotWheel->error, robotWheel->dt_ms);
            robotWheel->newTarget = robotWheel->targetSpeed + robotWheel->correction;

            // convert from rad/s to 0-255
            robotWheel->basePWMTarget = rad_s_to_pwm_value(robotWheel->targetSpeed);
            robotWheel->newPWMTarget = rad_s_to_pwm_value(robotWheel->newTarget);

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
        Serial.print(">encoderValue:");
        Serial.println(robotWheel->encoder_value);
        Serial.print(">encoderSpeed:");
        Serial.println(robotWheel->encoder_speed);
        Serial.print(">currentSpeed:");
        Serial.println(robotWheel->currentSpeed);
        Serial.print(">targetSpeed: ");
        Serial.println(robotWheel->targetSpeed);  
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