#include <RobotBaseDC.hpp>
#include <cmath>

int target_rad_s_to_pwm_command(float speed_rad_s)
{
    int absValue = 
        std::min(
            (std::abs(speed_rad_s) > MOTOR_ST0P_THRESHOLD_RAD_S ? MOTOR_TARGET_CONTROL_B : 0) + // feedforward control
            + 
            (int)(
                std::abs(RAD_S_TO_RPM(speed_rad_s)) 
                    * 255.0f / MAX_SPEED_RPM // scale 255 to max RPM
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

RobotWheelDC::RobotWheelDC(
    uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2, 
            uint8_t pinEncoderA, uint8_t pinEncoderB,
            std::string prefix,
            uint8_t pwmChannel) :
            AbstractRobotWheel(prefix), pinEncoderA_(pinEncoderA), pinEncoderB_(pinEncoderB),
            // explicitely initialize volatile values
            encoderValue_(0),
            oldTimeEncoderSpeed_(0)
{
    motorDriver = new L298N(
        pinEnable, 
        pinIN1, 
        pinIN2,
        pwmChannel
    );
    motorPID = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
        0.5 * 255.0 / VELOCITY_KP // max integral is 50% of the max control
    );
}

void RobotWheelDC::handleEncoderInterrupt()
{
    // Get current status.
    bool currentA = digitalRead(pinEncoderA_);

    // The direction of the encoder is given by currentA xor oldB
    encoderValue_ += (oldB ^ currentA ? 1 : -1);
    oldB = digitalRead(pinEncoderB_);
}

void RobotWheelDC::setWheelSpeed(float speed)
{
    targetSpeed_ = speed;

    // reset PID integral if target speed is zero to stop robot
    if (std::abs(speed) < 1e-3)
    {
        motorPID->resetIntegral();
    }
}

float RobotWheelDC::getWheelSpeed()
{
    return currentSpeed_;
}

void RobotWheelDC::updateMotorControl()
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

void RobotWheelDC::updateEncoderSpeed()
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

void RobotWheelDC::printToSerial()
{
    AbstractRobotWheel::printPrefix("encoderValue");
    Serial.println(encoderValue_);
    AbstractRobotWheel::printPrefix("currentSpeed");
    Serial.println(currentSpeed_);
    AbstractRobotWheel::printPrefix("targetSpeed");
    Serial.println(targetSpeed_);  
    AbstractRobotWheel::printPrefix("error");
    Serial.println(error_);       
    AbstractRobotWheel::printPrefix("correction");
    Serial.println(PWMcorrection_);        
    AbstractRobotWheel::printPrefix("basePWMTarget");
    Serial.println(basePWMTarget_);
    AbstractRobotWheel::printPrefix("newPWMTarget");
    Serial.println(newPWMTarget_);
    AbstractRobotWheel::printPrefix("dt_ms");
    Serial.println(dt_ms_);
}


// RobotBaseDC

void IRAM_ATTR encoderInterruptLeft()
{
    static_cast<RobotBaseDC*>(RobotBaseDC::getInstance())->leftWheel_->handleEncoderInterrupt();
}

void IRAM_ATTR encoderInterruptRight()
{
    static_cast<RobotBaseDC*>(RobotBaseDC::getInstance())->rightWheel_->handleEncoderInterrupt();
}

RobotBaseDC::RobotBaseDC()
{
    rightWheel_ = new RobotWheelDC(EN_A, IN1_A, IN2_A, ENCODER_A1, ENCODER_B1, "left_", 2);
    leftWheel_ = new RobotWheelDC(EN_B, IN1_B, IN2_B, ENCODER_B2, ENCODER_A2, "right_", 4);
}

void RobotBaseDC::setup()
{
    pinMode(leftWheel_->pinEncoderA_, INPUT_PULLUP);
    pinMode(leftWheel_->pinEncoderB_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(leftWheel_->pinEncoderA_), encoderInterruptLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftWheel_->pinEncoderB_), encoderInterruptLeft, CHANGE);

    pinMode(rightWheel_->pinEncoderA_, INPUT_PULLUP);
    pinMode(rightWheel_->pinEncoderB_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rightWheel_->pinEncoderA_), encoderInterruptRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightWheel_->pinEncoderB_), encoderInterruptRight, CHANGE);
}

void RobotBaseDC::setBaseSpeed(DrivetrainTarget target)
{
    leftWheel_->setWheelSpeed(target.motorSpeed[side::LEFT]);
    rightWheel_->setWheelSpeed(target.motorSpeed[side::RIGHT]);
}

void RobotBaseDC::updateControl() 
{
    leftWheel_->updateMotorControl();
    rightWheel_->updateMotorControl();
}

void RobotBaseDC::updateSensors() 
{
    leftWheel_->updateEncoderSpeed();
    rightWheel_->updateEncoderSpeed();
}

DrivetrainMeasurements RobotBaseDC::getMeasurements()
{
    DrivetrainMeasurements measurements;
    measurements.motorSpeed[side::LEFT] = leftWheel_->getWheelSpeed();
    measurements.motorSpeed[side::RIGHT] = rightWheel_->getWheelSpeed();
    return measurements;
}

RobotParameters RobotBaseDC::getParameters()
{
    RobotParameters parameters;
    parameters.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S;
    parameters.maxWheelSpeed = MAX_WHEEL_SPEED_MM_S;
    parameters.wheelRadius = WHEEL_RADIUS_MM;
    parameters.wheelSpacing = WHEEL_SPACING_MM;
    return parameters;
}

AbstractRobotBase* RobotBaseDC::getInstance()
{
    static RobotBaseDC instance;
    return &instance;
}