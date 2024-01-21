#include "../../include/parameters.hpp"
#ifdef USE_STEPPER_MOTORS
// #include "ESP32TimerInterrupt.h"
#include "RobotBaseStepper.hpp"
#include <cmath>
#include "FastAccelStepper.h"

/////////////////////////////////////////////
// Pinout
/////////////////////////////////////////////

#define STEP_1 13
#define DIR_1 14
#define STEP_2 27
#define DIR_2 23
#define NOT_ENABLE 4

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_RATED_RPM 120.0f // 30.0f
#define MOTOR_MICROSTEPS 16.0f
#define MOTOR_STEPS_PER_REVOLUTION (400.0f * MOTOR_MICROSTEPS)

#define rad_s_to_step_s(speed_rad_s) ((speed_rad_s) / (2.0f * M_PI) * MOTOR_STEPS_PER_REVOLUTION)
#define step_s_to_rad_s(speed_step_s) ((speed_step_s) * (2.0f * M_PI) / MOTOR_STEPS_PER_REVOLUTION)

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

#define WHEEL_RADIUS_MM 30.0f
#define WHEEL_SPACING_MM 31.0f

// give 20% overhead
#define MAX_SPEED_RPM (MOTOR_RATED_RPM)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))
#define MAX_SPEED_STEP_S (rad_s_to_step_s(MAX_SPEED_RAD_S))

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 200.0f

// very high value so the motor instantly accelerates
// acceleration constraints are taken into account by high level planner
#define MAX_MOTOR_ACCELERATION_STEPS_S 200000 // ((uint32_t)rad_s_to_step_s(200.0f / WHEEL_RADIUS_MM))


/////////////////////////////////////////////
// Wheel PID
/////////////////////////////////////////////

// convert from encoder pulse to encoder revolution
// convert from revolution to rad

// RobotBaseStepper

// volatile uint32_t Timer0Count = 0;

// volatile long timeFaster = 0;
// volatile unsigned long _lastStepTime_left = 0;
// volatile unsigned long _lastStepTime_right = 0;

// volatile long _currentPos_left = 0;
// volatile long _currentPos_right = 0;

// volatile unsigned long _stepInterval_left = 0;
// volatile unsigned long _stepInterval_right = 0;

// volatile bool _direction_left = true;
// volatile bool _direction_right = true;

// bool IRAM_ATTR TimerHandler0(void * timerNo)
// {

// 	// // Flag for checking to be sure ISR is working as Serial.print is not OK here in ISR
// 	// Timer0Count++;

//     if (_stepInterval_left == 0 && _stepInterval_right == 0)
//         return true;

//     timeFaster = micros(); 
//     bool mustStep_left = (_stepInterval_left > 0) && (timeFaster - _lastStepTime_left >= _stepInterval_left);
//     bool mustStep_right = (_stepInterval_right > 0) && (timeFaster - _lastStepTime_right >= _stepInterval_right);

//     // set pins
//     // GPIO.out_w1ts = (mustStep_right << STEP_1) | (mustStep_left << STEP_2);
//     if (mustStep_right)
//         digitalWrite(STEP_1, HIGH);
//     if (mustStep_left)
//         digitalWrite(STEP_2, HIGH);

//     // during which increments currentpos
//     _currentPos_left += mustStep_left ? (_direction_left ? 1 : -1) : 0;
//     _currentPos_right += mustStep_right ? (_direction_right ? 1 : -1) : 0;
//     if (mustStep_left)
// 	    _lastStepTime_left = timeFaster; // Caution: does not account for costs in step()
//     if (mustStep_right)
// 	    _lastStepTime_right = timeFaster; // Caution: does not account for costs in step()

//     // could be necessary in case loop is too fast for driver to react
//     delayMicroseconds(1);

//     // set pins
//     // GPIO.out_w1tc = (mustStep_right << STEP_1) | (mustStep_left << STEP_2);
//     if (mustStep_right)
//         digitalWrite(STEP_1, LOW);
//     if (mustStep_left)
//         digitalWrite(STEP_2, LOW);

   
//     return true;
// }

// ESP32Timer ITimer0(0);

// // Trigger motor driver enabled
// // bool isEnabled = true;
// void setMotorEnabled(bool enable)
// {
// //   if (isEnabled != enable)
// //   {
//     digitalWrite(NOT_ENABLE, !enable);
//     // isEnabled = enable;
// //   }
// }

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *leftStepper = NULL;
FastAccelStepper *rightStepper = NULL;

RobotBaseStepper::RobotBaseStepper()
{
    // motorPID_left = new miam::PID(
    //     VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
    //     0.5 * 100 / VELOCITY_KP // max integral is 50% of the max control
    // );
    // motorPID_right = new miam::PID(
    //     VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 
    //     0.5 * 100 / VELOCITY_KP // max integral is 50% of the max control
    // );


}

void RobotBaseStepper::setup()
{
    engine.init(1);
    leftStepper = engine.stepperConnectToPin(STEP_1, DRIVER_RMT);
    if (leftStepper) {
        leftStepper->setDirectionPin(DIR_1);
        leftStepper->setEnablePin(NOT_ENABLE);
        leftStepper->setAutoEnable(true);
        leftStepper->setDelayToEnable(10);
        leftStepper->setDelayToDisable(10);
        leftStepper->setAcceleration(MAX_MOTOR_ACCELERATION_STEPS_S * 2);
        leftStepper->setJumpStart(250);
        // leftStepper->setLinearAcceleration(0);
        // leftStepper->enableOutputs();
    } else {
        Serial.println("leftStepper failed");
    }
    rightStepper = engine.stepperConnectToPin(STEP_2, DRIVER_RMT);
    if (rightStepper) {
        rightStepper->setDirectionPin(DIR_2);
        rightStepper->setEnablePin(NOT_ENABLE);
        rightStepper->setAutoEnable(true);
        rightStepper->setDelayToEnable(10);
        rightStepper->setDelayToDisable(10);
        rightStepper->setAcceleration(MAX_MOTOR_ACCELERATION_STEPS_S * 2);
        rightStepper->setJumpStart(250);
        // rightStepper->setLinearAcceleration(0);
        // rightStepper->enableOutputs();
    } else {
        Serial.println("rightStepper failed");
    }

    // Configure pins
    // pinMode(STEP_1, OUTPUT);
    // pinMode(STEP_2, OUTPUT);
    // pinMode(DIR_1, OUTPUT);
    // pinMode(DIR_2, OUTPUT);

    // pinMode(NOT_ENABLE, OUTPUT);
    // setMotorEnabled(true);

    // vTaskDelay(100 / portTICK_PERIOD_MS);

    // Serial.print("max control to stepper=");
    // Serial.println(target_rad_s_to_stepper_speed(MAX_SPEED_RAD_S));

    // // Configure steper interrupt
    // // attach timer interrupt on core 1
    // if (ITimer0.attachInterruptInterval(50, TimerHandler0))
    //     {
    //         Serial.print(F("Starting  ITimer0 OK, millis() = "));
    //         Serial.println(millis());
    //     }
    //     else
    //         Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
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
    // setMotorEnabled(target.motorSpeed.norm() > 1e-6);


    // Set speed
    targetSpeed_left_ = target.motorSpeed[side::LEFT];
    targetSpeed_right_ = target.motorSpeed[side::RIGHT];
}

enum StepperState {
    FORWARD,
    BACKWARD,
    STOP
};

StepperState leftStepperState = STOP;
StepperState rightStepperState = STOP;

bool lastQueueWasDesyncLeft = false;
bool lastQueueWasDesyncRight = false;

int8_t applyTargetToStepper(int target, FastAccelStepper* stepper, StepperState& stepperState, bool reverseDirection, bool& lastQueueWasDesync, bool forceUpdate)
{
    if (reverseDirection)
        target = -target;
    
    if (std::abs(target) == 0)
    {
        if (stepperState == StepperState::STOP)
        {
            return 0;
        }
        else
        {
            stepper->stopMove();
            stepperState = STOP;
            return 0;
        }
    }

    // Increase target value
    // if (std::abs(target) > 0 && std::abs(target*1000) <= (1000LL * TICKS_PER_S / 0xffffffff + 1))
    // {
    //     target = (target > 0 ? 1 : -1) * ((1000LL * TICKS_PER_S / 0xffffffff + 1) + 1);
    // }
    if (std::abs(target) > 0 && std::abs(target) < 100)
    {
        target = (target > 0 ? 1 : -1) * 100;
    }

    int8_t result = stepper->setSpeedInHz((uint32_t)std::abs(target));
    
    // case need stop
    if (stepperState != StepperState::STOP && (result < 0 || (std::abs(target) < 10)) ) {
        // stepper->stopMove();
        stepperState = STOP;
    }
    else 
    {
        lastQueueWasDesync = (stepper->isQueueRunning() != (stepperState != StepperState::STOP)) || forceUpdate;
        if (target > 0)
        {
            if (lastQueueWasDesync || stepperState != StepperState::FORWARD) {
                result = stepper->runForward();
                if (result >= 0)
                {
                    stepperState = StepperState::FORWARD;
                    lastQueueWasDesync = false;
                }
            }
            else 
            {
                // if (!lastQueueWasDesync && (stepper->isQueueRunning() != (stepperState != StepperState::STOP)))
                // {
                //     lastQueueWasDesync = true;
                // }
                stepper->applySpeedAcceleration();
            }
        }
        else if (target < 0)
        {
            if (lastQueueWasDesync || stepperState != StepperState::BACKWARD) {
                result = stepper->runBackward();
                if (result >= 0)
                {
                    stepperState = StepperState::BACKWARD;
                    lastQueueWasDesync = false;
                }
            }
            else 
            {        
                // if (!lastQueueWasDesync && (stepper->isQueueRunning() != (stepperState != StepperState::STOP)))
                // {
                //     lastQueueWasDesync = true;
                // }
                stepper->applySpeedAcceleration();
            }
        }
    }
    return result;
}


void RobotBaseStepper::updateControl() 
{
    currentTime_ = micros();
    if (timeLowLevel_ > 0)
    {
        // dt_ms_ = (currentTime_ - timeLowLevel_) / 1000.0; // in ms
        // // error_left_ = currentSpeed_left_ - targetSpeed_left_; // in rad/s
        // // error_right_ = currentSpeed_right_ - targetSpeed_right_; // in rad/s

        // // correction_left_ = motorPID_left->computeValue(error_left_, dt_ms_);
        // // correction_right_ = motorPID_right->computeValue(error_right_, dt_ms_);

        // // convert from rad/s to 0-255
        // baseTarget_left_ = rad_s_to_step_s(targetSpeed_left_);
        // baseTarget_left_ = constrain(baseTarget_left_, -MAX_SPEED_STEP_S, MAX_SPEED_STEP_S);
        // // newTarget_left_ = round(baseTarget_left_ + correction_left_);
        // // newTarget_left_ = (newTarget_left_ > 0 ? 1 : -1) * std::min(std::abs(newTarget_left_), _maxSpeed);
        // newTarget_left_ = baseTarget_left_;

        // baseTarget_right_ = rad_s_to_step_s(targetSpeed_right_);
        // baseTarget_right_ = constrain(baseTarget_right_, -MAX_SPEED_STEP_S, MAX_SPEED_STEP_S);
        // // newTarget_right_ = round(baseTarget_right_ + correction_right_);
        // // newTarget_right_ = (newTarget_right_ > 0 ? 1 : -1) * std::min(std::abs(newTarget_right_),_maxSpeed);
        // newTarget_right_ = baseTarget_right_;

        baseTarget_left_ = rad_s_to_step_s(targetSpeed_left_);
        baseTarget_right_ = rad_s_to_step_s(targetSpeed_right_);

        // if (std::abs(baseTarget_left_*1000) > 0 && std::abs(baseTarget_left_*1000) <= (1000LL * TICKS_PER_S / 0xffffffff + 1))
        // {
        //     baseTarget_left_ = (baseTarget_left_ > 0 ? 1 : -1) * ((1000LL * TICKS_PER_S / 0xffffffff + 1) + 1);
        // }

        // if (std::abs(baseTarget_right_*1000) > 0 && std::abs(baseTarget_right_*1000) <= (1000LL * TICKS_PER_S / 0xffffffff + 1))
        // {
        //     baseTarget_right_ = (baseTarget_right_ > 0 ? 1 : -1) * ((1000LL * TICKS_PER_S / 0xffffffff + 1) + 1);
        // }

        // leftStepperResult_ = leftStepper->setSpeedInHz((uint32_t)std::abs(baseTarget_left_));
        // rightStepperResult_ = rightStepper->setSpeedInHz((uint32_t)std::abs(baseTarget_right_));

        leftStepperResult_ = applyTargetToStepper(baseTarget_left_, leftStepper, leftStepperState, true, lastQueueWasDesyncLeft, std::abs(currentSpeed_left_ - targetSpeed_left_) > 0.25);
        rightStepperResult_ = applyTargetToStepper(baseTarget_right_, rightStepper, rightStepperState, false, lastQueueWasDesyncRight, std::abs(currentSpeed_right_ - targetSpeed_right_) > 0.25);

        // // // leftStepper->setSpeedInMilliHz((uint32_t)std::abs(newTarget_left_*1000));
        // // if(leftStepper->isRunning() && leftStepperResult_ < 0 && !leftStepper->isStopping()) 
        // // {
        // //     // Serial.println("Failed apply left speed");
        // //     leftStepper->forceStopAndNewPosition(0);
        // //     leftStepperState = STOP;
        // // } 
        // // else 
        // {
        //     if (std::abs(baseTarget_left_) < 1e-6) {

        //     }
        //     if (!leftStepper->isStopping() && leftStepperState != STOP) 
        //     {
        //         leftStepper->forceStopAndNewPosition(0);
        //         leftStepperState = STOP;
        //     }
        //     else if (baseTarget_left_ > 0) 
        //     {
        //         // if (leftStepperState != FORWARD) 
        //         if (leftStepperState != FORWARD && leftStepper->runForward() == 0) 
        //         {
        //             // digitalWrite(DIR_1, HIGH);
        //             // leftStepper->runForward();
        //             leftStepperState = FORWARD;
        //         }
        //         // leftStepper->keepRunning();
        //         leftStepper->applySpeedAcceleration();
        //     }
        //     else if (baseTarget_left_ < 0)
        //     {
        //         // if (leftStepperState != BACKWARD) 
        //         if (leftStepperState != BACKWARD && leftStepper->runBackward() == 0) 
        //         {
        //             // digitalWrite(DIR_1, LOW);
        //             // leftStepper->runForward();
        //             leftStepperState = BACKWARD;
        //         }
        //         // leftStepper->keepRunning();
        //         leftStepper->applySpeedAcceleration();
        //     }
        // }
        // // leftStepper->applySpeedAcceleration();

        // // // rightStepper->setSpeedInMilliHz((uint32_t)std::abs(newTarget_right_*1000));
        // // if(rightStepper->isRunning() && rightStepperResult_ < 0 && !rightStepper->isStopping()) 
        // // {
        // //     // Serial.println("Failed apply right speed");
        // //     rightStepper->forceStopAndNewPosition(0);
        // //     rightStepperState = STOP;
        // // } 
        // // else 
        // {
        //     if (std::abs(baseTarget_right_) < 1e-6 && !rightStepper->isStopping() && rightStepperState != STOP) 
        //     {
        //         rightStepper->forceStopAndNewPosition(0);
        //         rightStepperState = STOP;
        //     }
        //     else if (baseTarget_right_ > 0) 
        //     {
        //         // if (rightStepperState != FORWARD) 
        //         if (rightStepperState != FORWARD && rightStepper->runForward() == 0) 
        //         {
        //             // digitalWrite(DIR_2, HIGH);
        //             // rightStepper->runForward();
        //             rightStepperState = FORWARD;
        //         }
        //         // rightStepper->keepRunning();
        //         rightStepper->applySpeedAcceleration();
        //     }
        //     else if (baseTarget_right_ < 0)
        //     {
        //         // if (rightStepperState != BACKWARD) 
        //         if (rightStepperState != BACKWARD && rightStepper->runBackward() == 0) 
        //         {
        //             // digitalWrite(DIR_2, LOW);
        //             // rightStepper->runForward();
        //             rightStepperState = BACKWARD;
        //         }
        //         // rightStepper->keepRunning();
        //         rightStepper->applySpeedAcceleration();
        //     }
        // }

        // rightStepper->applySpeedAcceleration();

        

        

        // // set step interval and direction
        // if (newTarget_left_ == 0.0)
        //     _stepInterval_left = 0;
        // else
        // {
        //     _stepInterval_left = fabs(1000000.0f / newTarget_left_);
        //     _direction_left = (newTarget_left_ > 0.0) ? true : false;
        //     digitalWrite(DIR_2, _direction_left);
        // }
        // if (newTarget_right_ == 0.0)
        //     _stepInterval_right = 0;
        // else
        // {
        //     _stepInterval_right = fabs(1000000.0f / newTarget_right_);
        //     _direction_right = (newTarget_right_ > 0.0) ? true : false;
        //     digitalWrite(DIR_1, _direction_right);
        // }
    }

    timeLowLevel_ = currentTime_;
}

// long timeFromLastPrint = 0;

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
        // encoderValue_left_ = leftStepper->getCurrentPosition();
        // encoderValue_right_ = rightStepper->getCurrentPosition();

        // // if (currentTimeEncoderSpeed_ - timeFromLastPrint > 1e6) {
        // //     Serial.printf("isRunningLeft: %i\n", leftStepper->isRunning());
        // //     Serial.printf("isRunningRight: %i\n", rightStepper->isRunning());
        // //     Serial.printf("encoderValue_left_: %f\n", encoderValue_left_);
        // //     Serial.printf("encoderValue_right_: %f\n", encoderValue_right_);
        // //     timeFromLastPrint = currentTimeEncoderSpeed_;
        // // }
        // currentSpeed_left_ = step_s_to_rad_s(
        //     (encoderValue_left_ - oldEncoderValue_left_) * 1.0f
        //         / ((currentTimeEncoderSpeed_ - oldTimeEncoderSpeed_) / 1000000.0f) // convert from us to s
        // );

        // currentSpeed_right_ = step_s_to_rad_s(
        //     (encoderValue_right_ - oldEncoderValue_right_) * 1.0f
        //         / ((currentTimeEncoderSpeed_ - oldTimeEncoderSpeed_) / 1000000.0f) // convert from us to s
        // );

        // currentSpeed_left_ = (leftStepperState == StepperState::FORWARD ? 1 : (leftStepperState == StepperState::BACKWARD ? -1 : 0)) * step_s_to_rad_s(leftStepper->getCurrentSpeedInMilliHz(true) / 1000.0f);
        // currentSpeed_right_ = (rightStepperState == StepperState::FORWARD ? 1 : (rightStepperState == StepperState::BACKWARD ? -1 : 0)) * step_s_to_rad_s(rightStepper->getCurrentSpeedInMilliHz(true) / 1000.0f);
        currentSpeed_left_ = -step_s_to_rad_s(leftStepper->getCurrentSpeedInMilliHz(true) / 1000.0f);
        currentSpeed_right_ = step_s_to_rad_s(rightStepper->getCurrentSpeedInMilliHz(true) / 1000.0f);

        targetSpeedDriver_left_ = step_s_to_rad_s(leftStepper->getSpeedInMilliHz() / 1000.0f);
        targetSpeedDriver_right_ = step_s_to_rad_s(rightStepper->getSpeedInMilliHz() / 1000.0f);

        isRunningLeft = leftStepper->isQueueRunning();
        isRunningRight = rightStepper->isQueueRunning();
        isEmptyLeft = leftStepper->isQueueEmpty();
        isEmptyRight = rightStepper->isQueueEmpty();
        isGenActiveLeft = leftStepper->isRampGeneratorActive();
        isGenActiveRight = rightStepper->isRampGeneratorActive();
        isFullLeft = leftStepper->isQueueFull();
        isFullRight = rightStepper->isQueueFull();

        desyncDetectedLeft = leftStepper->isQueueRunning() != (leftStepperState != STOP); 
        desyncDetectedRight = rightStepper->isQueueRunning() != (rightStepperState != STOP);
        
    }
    oldTimeEncoderSpeed_ = currentTimeEncoderSpeed_;
    // oldEncoderValue_left_ = encoderValue_left_;
    // oldEncoderValue_right_ = encoderValue_right_;
}


// unsigned long RobotBaseStepper::getStepIntervalRight()
// {
//     return _stepInterval_right;
// }


// unsigned long RobotBaseStepper::getStepIntervalLeft()
// {
//     return _stepInterval_left;
// }


void RobotBaseStepper::forceStop() 
{
    // setMotorEnabled(false);
    leftStepper->forceStop();
    rightStepper->forceStop();
}
#endif