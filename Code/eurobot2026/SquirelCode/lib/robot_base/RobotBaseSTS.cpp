#include "../../include/parameters.hpp"
#include <RobotBaseSTS.hpp>
#include <cmath>
#include <STSServoDriver.h>

#define SERVO_DIR_PIN 5
#define RXD2 16
#define TXD2 17

STSServoDriver servos;
HardwareSerial serial2(2);

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define SERVO_STEP_PER_TURN 5000
#define SERVO_MAX_STEPS_S 10000

#define MOTOR_RATED_RPM 50.0f

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

// #if PAMI_ID == 5
// #define WHEEL_RADIUS_MM 13.5f
// #define WHEEL_SPACING_MM 32.5f
// #else 
#define WHEEL_RADIUS_MM 30.0f
#define WHEEL_SPACING_MM (104.0f / 2.0f)
// #endif

// #if PAMI_ID == 5
//     #define WHEEL_SPACING_MM 42.5f
// #else 
//     //41.0f
// #endif

// give 20% overhead
#define MAX_SPEED_RPM (MOTOR_RATED_RPM * 0.75)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 300.0f

/////////////////////////////////////////////
// Wheel PID
/////////////////////////////////////////////

// Target control will be
// PWM = targetSpeed (rpm) * 255 / max speed (rpm) * MOTOR_TARGET_CONTROL_A + MOTOR_TARGET_CONTROL_B
#define MOTOR_TARGET_CONTROL_B 20
#define MOTOR_TARGET_CONTROL_A 1.0f
// offset will be applied if targetSpeed (rad/s) is above threshold
#define MOTOR_ST0P_THRESHOLD_RAD_S 0.02f

// Wheel PID parameters
#define VELOCITY_KP 0.05f
#define VELOCITY_KD 0.0f
#define VELOCITY_KI 0.1f

int target_rad_s_to_steps_s(float speed_rad_s)
{
    return
        speed_rad_s 
            / (2.0 * M_PI) 
            * SERVO_STEP_PER_TURN;
    }

float servo_step_s_to_rad_s(int step_s)
{
    return
        step_s
            / (1.0*SERVO_STEP_PER_TURN)
            * (2.0 * M_PI);
}

SemaphoreHandle_t servoSemaphore = NULL;

RobotWheelSTS::RobotWheelSTS(
    byte servo_id,
    std::string prefix,
    bool inverted
) : AbstractRobotWheel(prefix), servo_id_(servo_id), inverted_(inverted)
{
    motorPID = new miam::PID(
        VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 2.0
    );
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        // Set the servo to velocity mode.
        servos.setMode(servo_id_, STSMode::VELOCITY);
        xSemaphoreGive(servoSemaphore);
    }
}

void RobotWheelSTS::updateMotorControl(bool motorEnabled)
{
    currentTime_ = micros();
    if (timeLowLevel_ > 0)
    {
        dt_ms_ = (currentTime_ - timeLowLevel_) / 1000.0; // in ms

        // case motors should not move
        if (!motorEnabled)
        {
            error_ = 0.0f;
            correction_ = 0.0f;
            baseTarget_ = 0;
            newTarget_ = 0;
            motorPID->resetIntegral();
        }
        else
        {
            error_ = currentSpeed_ - targetSpeed_; // in rad/s
            correction_ = motorPID->computeValue(error_, dt_ms_);
            
            // convert from rad/s to 0-255
            baseTarget_ = target_rad_s_to_steps_s(targetSpeed_);
            newTarget_ = target_rad_s_to_steps_s(targetSpeed_ + correction_);
            newTarget_ = (newTarget_ > 0 ? 1 : -1) * std::min(std::abs(newTarget_), SERVO_MAX_STEPS_S);

        }
        if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
            // inverted ?
            int targetToSend = newTarget_;
            if (inverted_)
            {
                targetToSend = -targetToSend;
            }
            servos.setTargetVelocity(servo_id_, targetToSend);
            xSemaphoreGive(servoSemaphore);
        }
    }
    timeLowLevel_ = currentTime_;
}

void RobotWheelSTS::printToSerial()
{
    AbstractRobotWheel::printPrefix("currentSpeed");
    Serial.println(currentSpeed_);
    AbstractRobotWheel::printPrefix("targetSpeed");
    Serial.println(targetSpeed_);  
    AbstractRobotWheel::printPrefix("error");
    Serial.println(error_);       
    AbstractRobotWheel::printPrefix("correction");
    Serial.println(correction_);        
    AbstractRobotWheel::printPrefix("basePWMTarget");
    Serial.println(baseTarget_);
    AbstractRobotWheel::printPrefix("newPWMTarget");
    Serial.println(newTarget_);
    AbstractRobotWheel::printPrefix("dt_ms");
    Serial.println(dt_ms_);
}

void RobotWheelSTS::updateEncoderSpeed()
{
    
    int currentSpeed = 0;
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        currentSpeed = servos.getCurrentSpeed(servo_id_);
        xSemaphoreGive(servoSemaphore);
    }
    // inverted ?
    if (inverted_)
    {
        currentSpeed = -currentSpeed;
    }
    currentSpeed_ = servo_step_s_to_rad_s(currentSpeed);
}

float RobotWheelSTS::getWheelSpeed()
{
    return currentSpeed_;
}


// RobotBaseSTS

RobotBaseSTS::RobotBaseSTS()
{
    servoSemaphore = xSemaphoreCreateMutex();
    serial2.setPins(RXD2, TXD2);
    
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        servos.init(SERVO_DIR_PIN, &serial2);
        xSemaphoreGive(servoSemaphore);
    }
    rightWheel_ = new RobotWheelSTS(1, "right_", false);
    leftWheel_ = new RobotWheelSTS(2, "left_", true);
}

void RobotBaseSTS::setup()
{
}

DrivetrainMeasurements RobotBaseSTS::getMeasurements()
{
    DrivetrainMeasurements measurements;
    measurements.motorSpeed[side::LEFT] = leftWheel_->getWheelSpeed();
    measurements.motorSpeed[side::RIGHT] = rightWheel_->getWheelSpeed();
    return measurements;
}

RobotParameters RobotBaseSTS::getParameters()
{
    RobotParameters parameters;
    parameters.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S;
    parameters.maxWheelSpeed = MAX_WHEEL_SPEED_MM_S;
    parameters.wheelRadius = WHEEL_RADIUS_MM;
    parameters.wheelSpacing = WHEEL_SPACING_MM;
    return parameters;
}

AbstractRobotBase* RobotBaseSTS::getInstance()
{
    static RobotBaseSTS instance;
    return &instance;
}