#include <RobotWheel.hpp>
#include <parameters.hpp>
#include <cmath>

int target_rad_s_to_pwm_command(double speed_rad_s)
{
    int absValue = 
        std::min(
            (std::abs(speed_rad_s) > MOTOR_ST0P_THRESHOLD_RAD_S ? MOTOR_TARGET_CONTROL_B : 0) + // feedforward control
            + (int)(
                std::abs(RAD_S_TO_RPM(speed_rad_s)) 
                    * 255.0 / MAX_SPEED_RPM // scale 255 to max RPM
                    * MOTOR_TARGET_CONTROL_A // comes from experience
                ), 
            255 // cap to 255
        );
    return(
        (speed_rad_s < 0 ? -1.0 : 1.0) // sign
            * absValue
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
            uint8_t pinEncoderA, uint8_t pinEncoderB,
            std::string prefix) :
            pinEncoderA_(pinEncoderA), pinEncoderB_(pinEncoderB),
            prefix_(prefix),
            // explicitely initialize volatile values
            encoderValue_(0), encoderSpeed_(0),
            currentSpeed_(0), targetSpeed_(0)
{
    motorDriver = new L298N(
        pinEnable, 
        pinIN1, 
        pinIN2
    );
    motorPID = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
        0.5 * 255.0 / VELOCITY_KP // max integral is 50% of the max control
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
    vTaskDelay(3 / portTICK_PERIOD_MS);

    xTaskCreate(
        RobotWheel::tickMotorControl, 
        "tickMotorControl",
        10000,
        this,
        1,
        NULL
    ); 
    vTaskDelay(3 / portTICK_PERIOD_MS);
}

void RobotWheel::handleEncoderInterrupt()
{
    // Get current status.
    bool currentA = digitalRead(pinEncoderA_);

    // The direction of the encoder is given by currentA xor oldB
    encoderValue_ += (oldB ^ currentA ? 1 : -1);
    oldB = digitalRead(pinEncoderB_);
}

void RobotWheel::setWheelSpeed(double speed)
{
    targetSpeed_ = speed;

    // reset PID integral if target speed is zero to stop robot
    if (std::abs(speed) < 1e-6)
    {
        motorPID->resetIntegral();
    }
}

double RobotWheel::getWheelSpeed()
{
    return currentSpeed_;
}

void RobotWheel::tickMotorControl(void* parameters)
{
    RobotWheel* robotWheel = static_cast<RobotWheel*>(parameters);

    for (;;)
    {
        robotWheel->currentTime_ = micros();
        if (robotWheel->timeLowLevel_ > 0)
        {
            robotWheel->dt_ms_ = (robotWheel->currentTime_ - robotWheel->timeLowLevel_) / 1000.0; // in ms
        
            // speed in rad/s
            robotWheel->currentSpeed_ = encoder_ticks_to_rad_s(robotWheel->encoderSpeed_);

            robotWheel->error_ = robotWheel->currentSpeed_ - robotWheel->targetSpeed_; // in rad/s

            robotWheel->PWMcorrection_ = robotWheel->motorPID->computeValue(robotWheel->error_, robotWheel->dt_ms_);

            // convert from rad/s to 0-255
            robotWheel->basePWMTarget_ = target_rad_s_to_pwm_command(robotWheel->targetSpeed_);
            robotWheel->newPWMTarget_ = round(robotWheel->basePWMTarget_ + robotWheel->PWMcorrection_);

            if (robotWheel->newPWMTarget_ >= 0)
            {
                robotWheel->motorDriver->forward();
            }
            else
            {
                robotWheel->motorDriver->backward();
            }
            robotWheel->motorDriver->setSpeed(std::abs(robotWheel->newPWMTarget_));
        }
        robotWheel->timeLowLevel_ = robotWheel->currentTime_;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }    
}

void RobotWheel::tickEncoderSpeed(void* parameters)
{
    RobotWheel* robotWheel = static_cast<RobotWheel*>(parameters);
    for (;;)
    {
        robotWheel->encoderSpeed_ = 
            robotWheel->encoderValue_ - robotWheel->oldEncoderValue_;
        robotWheel->oldEncoderValue_ = robotWheel->encoderValue_;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }    
}

void RobotWheel::printPrefix(const char* value_name)
{
    Serial.print(">");
    Serial.print(prefix_.c_str());
    Serial.print(value_name);
    Serial.print(":");
}

void RobotWheel::printToSerial()
{
    printPrefix("encoderValue");
    Serial.println(encoderValue_);
    printPrefix("encoderSpeed");
    Serial.println(encoderSpeed_);
    printPrefix("currentSpeed");
    Serial.println(currentSpeed_);
    printPrefix("targetSpeed");
    Serial.println(targetSpeed_);  
    printPrefix("error");
    Serial.println(error_);       
    printPrefix("correction");
    Serial.println(PWMcorrection_);        
    printPrefix("basePWMTarget");
    Serial.println(basePWMTarget_);
    printPrefix("newPWMTarget");
    Serial.println(newPWMTarget_);
    printPrefix("dt_ms");
    Serial.println(dt_ms_);
}