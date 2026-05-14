#include <Arduino.h>
#include <ServoHandler.hpp>
#include <RobotServos.hpp>

#define SERVO_3_POSITION_FOLD 650
#define SERVO_3_POSITION_UP 650
#define SERVO_3_POSITION_UP_WITH_CRATE 600
#define SERVO_3_POSITION_DOWN 830
#define SERVO_3_POSITION_MID (SERVO_3_POSITION_UP + SERVO_3_POSITION_DOWN) / 2

#define SERVO_4_POSITION_FOLD 800
#define SERVO_4_POSITION_UP 580
#define SERVO_4_POSITION_UP_WITH_CRATE 467
#define SERVO_4_POSITION_DOWN 467
#define SERVO_4_POSITION_MID (SERVO_4_POSITION_UP + SERVO_4_POSITION_DOWN) / 2

#define SERVO_3_POSITION_FUNNYACTION_UP 800
#define SERVO_3_POSITION_FUNNYACTION_DOWN 830

#define SERVO_4_POSITION_FUNNYACTION_UP 467
#define SERVO_4_POSITION_FUNNYACTION_DOWN 467

#define PUMP_GPIO 18

namespace ServoHandler
{
    void init()
    {
        RobotServos::init_servo_id_position(3);
        RobotServos::init_servo_id_position(4);
        pinMode(PUMP_GPIO, OUTPUT);
    }

    void armPositionUp()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_UP);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_UP);
        delay(10);
    }

    void armPositionUpWithCrate()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_UP_WITH_CRATE);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_UP_WITH_CRATE);
        delay(10);
    }


    void armPositionMid()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_MID);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_MID);
        delay(10);
    }

    void armPositionUpHorizontal()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_UP);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_DOWN);
        delay(10);
    }


    void armPositionDown()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_DOWN);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_DOWN);
        delay(10);
    }

    void armPositionFold()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_FOLD);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_FOLD);
        delay(10);
    }

    void armPositionFunnyUp()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_FUNNYACTION_UP);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_FUNNYACTION_UP);
        delay(10);
    }

    void armPositionFunnyDown()
    {
        RobotServos::set_servo_position(3, SERVO_3_POSITION_FUNNYACTION_DOWN);
        delay(10);
        RobotServos::set_servo_position(4, SERVO_4_POSITION_FUNNYACTION_DOWN);
        delay(10);
    }
    
    void pumpOn()
    {
        digitalWrite(PUMP_GPIO, HIGH);
    }

    void pumpOff()
    {
        digitalWrite(PUMP_GPIO, LOW);
    }
}

// // ServoHandler
// #define SERVO_PIN 25
// #define SERVO_PWM_CHANNEL 6
// #define SERVO_PIN_2 26
// #define SERVO_PWM_CHANNEL_2 7

// #define SERVO_UP_POSITION 50
// #define SERVO_DOWN_POSITION 75

// const int TIMER_RESOLUTION = std::min(16, SOC_LEDC_TIMER_BIT_WIDE_NUM);
// const int PERIOD_TICKS = (1 << TIMER_RESOLUTION) - 1;
// const int DEFAULT_FREQUENCY = 50;

// float _minAngle = 0.0;
// float _maxAngle = 180.0;
// int _minPulseWidthUs = 544;
// int _maxPulseWidthUs = 2400;
// int _periodUs = 1000000 / DEFAULT_FREQUENCY;

// float mapTemplate(float x, float in_min, float in_max, float out_min, float out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// int _angleToUs(float angle) {
//     return mapTemplate(angle, _minAngle, _maxAngle, _minPulseWidthUs, _maxPulseWidthUs);
// }

// int _usToTicks(int us) { return std::round((PERIOD_TICKS * us) / _periodUs); }
// int _ticksToUs(int duty) { return std::round((_periodUs * duty) / PERIOD_TICKS); }
// float _usToAngle(int us) { return mapTemplate(us, _minPulseWidthUs, _maxPulseWidthUs, _minAngle, _maxAngle); }

// void servoWriteMicroseconds(int pulseWidthUs, char servo_id) {
//     pulseWidthUs = constrain(pulseWidthUs, _minPulseWidthUs, _maxPulseWidthUs);
//     int _pulseWidthTicks = _usToTicks(pulseWidthUs);
//     if (servo_id == 1)
//         ledcWrite(SERVO_PWM_CHANNEL, _pulseWidthTicks);
//     else if (servo_id == 2)
//         ledcWrite(SERVO_PWM_CHANNEL_2, _pulseWidthTicks);
// }

// namespace ServoHandler
// {
//     void init()
//     {
//         ledcSetup(SERVO_PWM_CHANNEL, DEFAULT_FREQUENCY, TIMER_RESOLUTION);
//         ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
//         ledcSetup(SERVO_PWM_CHANNEL_2, DEFAULT_FREQUENCY, TIMER_RESOLUTION);
//         ledcAttachPin(SERVO_PIN_2, SERVO_PWM_CHANNEL_2);
//     }

//     void servoWrite(float angle, char servo_id) {
//         angle = constrain(angle, _minAngle, _maxAngle);
//         servoWriteMicroseconds(_angleToUs(angle), servo_id);
//     }

//     void servoUp()
//     {
//         servoWrite(SERVO_UP_POSITION, 1);
//         servoWrite(SERVO_UP_POSITION, 2);
//     }

//     void servoDown()
//     {
//         servoWrite(SERVO_DOWN_POSITION, 1);
//         servoWrite(SERVO_DOWN_POSITION, 2);
//     }
// }


