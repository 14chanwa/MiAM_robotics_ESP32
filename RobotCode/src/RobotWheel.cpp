#include <RobotWheel.hpp>
#include <parameters.hpp>
#include <cmath>

int target_rad_s_to_pwm_command(float speed_rad_s)
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

float encoder_pulse_s_to_rad_s(float encoder_ticks_s)
{
    return(
        encoder_ticks_s
            / ENCODER_PULSE_PER_REVOLUTION // convert from encoder pulse to encoder revolution
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
            encoderValue_(0),
            currentSpeed_(0), targetSpeed_(0),
            oldTimeEncoderSpeed_(0)
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

void RobotWheel::handleEncoderInterrupt()
{
    // Get current status.
    bool currentA = digitalRead(pinEncoderA_);

    // The direction of the encoder is given by currentA xor oldB
    encoderValue_ += (oldB ^ currentA ? 1 : -1);
    oldB = digitalRead(pinEncoderB_);
}

void RobotWheel::setWheelSpeed(float speed)
{
    targetSpeed_ = speed;

    // reset PID integral if target speed is zero to stop robot
    if (std::abs(speed) < 1e-6)
    {
        motorPID->resetIntegral();
    }
}

float RobotWheel::getWheelSpeed()
{
    return currentSpeed_;
}

void RobotWheel::updateMotorControl()
{
    currentTime_ = micros();
    if (timeLowLevel_ > 0)
    {
        dt_ms_ = (currentTime_ - timeLowLevel_) / 1000.0; // in ms
        error_ = currentSpeed_ - targetSpeed_; // in rad/s

        PWMcorrection_ = motorPID->computeValue(error_, dt_ms_);

        // convert from rad/s to 0-255
        basePWMTarget_ = target_rad_s_to_pwm_command(targetSpeed_);
        newPWMTarget_ = round(basePWMTarget_ + PWMcorrection_);
        newPWMTarget_ = (newPWMTarget_ > 0 ? 1 : -1) * std::min(std::abs(newPWMTarget_), 255);

        if (newPWMTarget_ >= 0)
        {
            motorDriver->forward();
        }
        else
        {
            motorDriver->backward();
        }
        motorDriver->setSpeed(std::abs(newPWMTarget_));
    }
    timeLowLevel_ = currentTime_;
}

void RobotWheel::updateEncoderSpeed()
{
    unsigned long currentTimeEncoderSpeed_ = micros();
    if (oldTimeEncoderSpeed_ == 0)
    {
        currentSpeed_ = 0;
    }
    else
    {
        currentSpeed_ = encoder_pulse_s_to_rad_s(
            (encoderValue_ - oldEncoderValue_) 
                / ((currentTimeEncoderSpeed_ - oldTimeEncoderSpeed_) / 1000000.0) // convert from us to s
        );
        
    }
    oldTimeEncoderSpeed_ = currentTimeEncoderSpeed_;
    oldEncoderValue_ = encoderValue_;
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