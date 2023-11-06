
#include "ESP32TimerInterrupt.h"
#include "RobotBaseStepper.hpp"
#include <cmath>

/////////////////////////////////////////////
// Pinout
/////////////////////////////////////////////

#define STEP_1 13
#define DIR_1 14
#define STEP_2 27
#define DIR_2 32
#define NOT_ENABLE 33

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_STEPS_PER_REVOLUTION 2048
#define MOTOR_RATED_RPM 30.0f

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

#define WHEEL_RADIUS_MM 31.0f
#define WHEEL_SPACING_MM 31.6f

// give 20% overhead
#define MAX_SPEED_RPM (MOTOR_RATED_RPM)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 50.0f


/////////////////////////////////////////////
// Wheel PID
/////////////////////////////////////////////

// Wheel PID parameters
#define VELOCITY_KP 0.2f
#define VELOCITY_KD 0.0f
#define VELOCITY_KI 0.0f

#define target_rad_s_to_stepper_speed(speed_rad_s) (speed_rad_s / 2.0f * M_PI) * MOTOR_STEPS_PER_REVOLUTION

#define _maxSpeed ((int)std::round(MOTOR_RATED_RPM * MOTOR_STEPS_PER_REVOLUTION / 60.0f))

// convert from encoder pulse to encoder revolution
// convert from revolution to rad
#define step_s_to_rad_s(encoder_ticks_s) encoder_ticks_s / MOTOR_STEPS_PER_REVOLUTION * 2.0 * M_PI 

// RobotBaseStepper

// volatile uint32_t Timer0Count = 0;

volatile long timeFaster = 0;
volatile unsigned long _lastStepTime_left = 0;
volatile unsigned long _lastStepTime_right = 0;

volatile long _currentPos_left = 0;
volatile long _currentPos_right = 0;

volatile unsigned long _stepInterval_left = 0;
volatile unsigned long _stepInterval_right = 0;

volatile bool _direction_left = true;
volatile bool _direction_right = true;

bool IRAM_ATTR TimerHandler0(void * timerNo)
{

	// // Flag for checking to be sure ISR is working as Serial.print is not OK here in ISR
	// Timer0Count++;

    if (_stepInterval_left == 0 && _stepInterval_right == 0)
        return true;

    timeFaster = micros(); 
    bool mustStep_left = (_stepInterval_left > 0) && (timeFaster - _lastStepTime_left >= _stepInterval_left);
    bool mustStep_right = (_stepInterval_right > 0) && (timeFaster - _lastStepTime_right >= _stepInterval_right);

    // set pins
    GPIO.out_w1ts = (mustStep_right << STEP_1) | (mustStep_left << STEP_2);
    // if (mustStep_right)
    //     digitalWrite(STEP_1, HIGH);
    // if (mustStep_left)
    //     digitalWrite(STEP_2, HIGH);

    // during which increments currentpos
    _currentPos_left += mustStep_left ? (_direction_left ? 1 : -1) : 0;
    _currentPos_right += mustStep_right ? (_direction_right ? 1 : -1) : 0;
    if (mustStep_left)
	    _lastStepTime_left = timeFaster; // Caution: does not account for costs in step()
    if (mustStep_right)
	    _lastStepTime_right = timeFaster; // Caution: does not account for costs in step()

    // could be necessary in case loop is too fast for driver to react
    // delayMicroseconds(1);

    // set pins
    GPIO.out_w1tc = (mustStep_right << STEP_1) | (mustStep_left << STEP_2);
    // if (mustStep_right)
    //     digitalWrite(STEP_1, LOW);
    // if (mustStep_left)
    //     digitalWrite(STEP_2, LOW);

   
    return true;
}

ESP32Timer ITimer0(0);

// Trigger motor driver enabled
// bool isEnabled = true;
void setMotorEnabled(bool enable)
{
//   if (isEnabled != enable)
//   {
    digitalWrite(NOT_ENABLE, !enable);
    // isEnabled = enable;
//   }
}

RobotBaseStepper::RobotBaseStepper()
{
    motorPID_left = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
        0.5 * 100 / VELOCITY_KP // max integral is 50% of the max control
    );
    motorPID_right = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
        0.5 * 100 / VELOCITY_KP // max integral is 50% of the max control
    );
}

void RobotBaseStepper::setup()
{
    // Configure pins
    pinMode(STEP_1, OUTPUT);
    pinMode(STEP_2, OUTPUT);
    pinMode(DIR_1, OUTPUT);
    pinMode(DIR_2, OUTPUT);

    pinMode(NOT_ENABLE, OUTPUT);
    setMotorEnabled(false);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    Serial.print("max control to stepper=");
    Serial.println(target_rad_s_to_stepper_speed(MAX_SPEED_RAD_S));

    // Configure steper interrupt
    // attach timer interrupt on core 1
    if (ITimer0.attachInterruptInterval(20, TimerHandler0))
        {
            Serial.print(F("Starting  ITimer0 OK, millis() = "));
            Serial.println(millis());
        }
        else
            Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
}

DrivetrainMeasurements RobotBaseStepper::getMeasurements()
{
    DrivetrainMeasurements measurements;
    measurements.motorSpeed[side::LEFT] = currentSpeed_left_;
    measurements.motorSpeed[side::RIGHT] = currentSpeed_right_;
    return measurements;
}

RobotParameters RobotBaseStepper::getParameters()
{
    RobotParameters parameters;
    parameters.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S;
    parameters.maxWheelSpeed = MAX_WHEEL_SPEED_MM_S;
    parameters.wheelRadius = WHEEL_RADIUS_MM;
    parameters.wheelSpacing = WHEEL_SPACING_MM;
    return parameters;
}

AbstractRobotBase* RobotBaseStepper::getInstance()
{
    static RobotBaseStepper instance;
    return &instance;
}

void RobotBaseStepper::setBaseSpeed(DrivetrainTarget target)
{
    // If speed is zero, deactivate steppers to save power
    setMotorEnabled(target.motorSpeed.norm() > 1e-6);

    // Set speed
    targetSpeed_left_ = target.motorSpeed[side::LEFT];
    targetSpeed_right_ = target.motorSpeed[side::RIGHT];
}

void RobotBaseStepper::updateControl() 
{
    currentTime_ = micros();
    if (timeLowLevel_ > 0)
    {
        dt_ms_ = (currentTime_ - timeLowLevel_) / 1000.0; // in ms
        error_left_ = currentSpeed_left_ - targetSpeed_left_; // in rad/s
        error_right_ = currentSpeed_right_ - targetSpeed_right_; // in rad/s

        correction_left_ = motorPID_left->computeValue(error_left_, dt_ms_);
        correction_right_ = motorPID_right->computeValue(error_right_, dt_ms_);

        // convert from rad/s to 0-255
        baseTarget_left_ = target_rad_s_to_stepper_speed(targetSpeed_left_);
        baseTarget_left_ = constrain(baseTarget_left_, -_maxSpeed, _maxSpeed);
        newTarget_left_ = round(baseTarget_left_ + correction_left_);
        newTarget_left_ = (newTarget_left_ > 0 ? 1 : -1) * std::min(std::abs(newTarget_left_), _maxSpeed);

        baseTarget_right_ = target_rad_s_to_stepper_speed(targetSpeed_right_);
        baseTarget_right_ = constrain(baseTarget_right_, -_maxSpeed, _maxSpeed);
        newTarget_right_ = round(baseTarget_right_ + correction_right_);
        newTarget_right_ = (newTarget_right_ > 0 ? 1 : -1) * std::min(std::abs(newTarget_right_),_maxSpeed);

        // set step interval and direction
        if (newTarget_left_ == 0.0)
            _stepInterval_left = 0;
        else
        {
            _stepInterval_left = fabs(1000000.0f / newTarget_left_);
            _direction_left = (newTarget_left_ > 0.0) ? true : false;
            digitalWrite(DIR_2, _direction_left);
        }
        if (newTarget_right_ == 0.0)
            _stepInterval_right = 0;
        else
        {
            _stepInterval_right = fabs(1000000.0f / newTarget_right_);
            _direction_right = (newTarget_right_ > 0.0) ? true : false;
            digitalWrite(DIR_1, _direction_right);
        }
    }

    timeLowLevel_ = currentTime_;
}

void RobotBaseStepper::updateSensors() 
{
    unsigned long currentTimeEncoderSpeed_ = micros();
    if (oldTimeEncoderSpeed_ == 0)
    {
        currentSpeed_left_ = 0;
        currentSpeed_right_ = 0;
    }
    else
    {
        encoderValue_left_ = _currentPos_left;
        encoderValue_right_ = _currentPos_right;
        currentSpeed_left_ = step_s_to_rad_s(
            (encoderValue_left_ - oldEncoderValue_left_) 
                / ((currentTimeEncoderSpeed_ - oldTimeEncoderSpeed_) / 1000000.0) // convert from us to s
        );

        currentSpeed_right_ = step_s_to_rad_s(
            (encoderValue_right_ - oldEncoderValue_right_) 
                / ((currentTimeEncoderSpeed_ - oldTimeEncoderSpeed_) / 1000000.0) // convert from us to s
        );
        
    }
    oldTimeEncoderSpeed_ = currentTimeEncoderSpeed_;
    oldEncoderValue_left_ = encoderValue_left_;
    oldEncoderValue_right_ = encoderValue_right_;
}


unsigned long RobotBaseStepper::getStepIntervalRight()
{
    return _stepInterval_right;
}


unsigned long RobotBaseStepper::getStepIntervalLeft()
{
    return _stepInterval_left;
}


void RobotBaseStepper::forceStop() 
{
    setMotorEnabled(false);
}