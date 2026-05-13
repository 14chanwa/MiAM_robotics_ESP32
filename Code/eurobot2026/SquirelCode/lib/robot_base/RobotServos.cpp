#include <RobotServos.hpp>

// Servo
#include <STSServoDriver.h>

#define SERVO_DIR_PIN 5
#define RXD2 16
#define TXD2 17

#include "esp_log.h"
static const char* TAG = "RobotServos.cpp";

#define DEBUG_PRINT(x) ESP_LOGI(TAG, "%s", x)
#define DEBUG_PRINTLN(x) ESP_LOGI(TAG, "%s", x)

STSServoDriver servos;
HardwareSerial serial2(2);
SemaphoreHandle_t servoSemaphore = NULL;

void RobotServos::init()
{
    servoSemaphore = xSemaphoreCreateMutex();
    serial2.setPins(RXD2, TXD2);
    
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        servos.init(SERVO_DIR_PIN, &serial2);
        xSemaphoreGive(servoSemaphore);
    }
}

void RobotServos::init_servo_id_velocity(byte servo_id)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        // Set the servo to velocity mode.
        servos.setMode(servo_id, STSMode::VELOCITY);
        xSemaphoreGive(servoSemaphore);
    }
}
void RobotServos::init_servo_id_position(byte servo_id)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        // Set the servo to position mode.
        servos.setMode(servo_id, STSMode::POSITION);
        xSemaphoreGive(servoSemaphore);
    }
}
void RobotServos::init_servo_id_step(byte servo_id)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        // Set the servo to position mode.
        servos.setMode(servo_id, STSMode::STEP);
        delay(100);
        servos.writeTwoBytesRegister(servo_id, STSRegisters::MAXIMUM_ANGLE, 0);
        delay(100);
        servos.setTargetVelocity(servo_id, 1000);
        delay(100);
        servos.setTargetAcceleration(servo_id, 10);
        delay(100);
        xSemaphoreGive(servoSemaphore);
    }
}
void RobotServos::set_servo_velocity(byte servo_id, int target_velocity)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        servos.setTargetVelocity(servo_id, target_velocity);
        xSemaphoreGive(servoSemaphore);
    }
}
void RobotServos::set_servo_position(byte servo_id, int target_position)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        servos.setTargetPosition(servo_id, target_position);
        xSemaphoreGive(servoSemaphore);
    }
}

int RobotServos::get_current_speed(byte servo_id)
{
    int currentSpeed = 0;
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        currentSpeed = servos.getCurrentSpeed(servo_id);
        xSemaphoreGive(servoSemaphore);
    }
    return currentSpeed;
}

int RobotServos::get_current_position(byte servo_id)
{
    int currentPosition = 0;
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        currentPosition = servos.getCurrentPosition(servo_id);
        xSemaphoreGive(servoSemaphore);
    }
    return currentPosition;
}

void RobotServos::stop(byte servo_id)
{
    if (xSemaphoreTake(servoSemaphore, portMAX_DELAY)) {
        servos.writeRegister(servo_id, STSRegisters::TORQUE_SWITCH, 0);
        xSemaphoreGive(servoSemaphore);
    }
}