#include <Arduino.h>
#include <ServoHandler.hpp>

// ServoHandler
#define SERVO_PIN 25
#define SERVO_PWM_CHANNEL 6

#define SERVO_UP_POSITION 10
#define SERVO_DOWN_POSITION 100

const int TIMER_RESOLUTION = std::min(16, SOC_LEDC_TIMER_BIT_WIDE_NUM);
const int PERIOD_TICKS = (1 << TIMER_RESOLUTION) - 1;
const int DEFAULT_FREQUENCY = 50;

float _minAngle = 0.0;
float _maxAngle = 180.0;
int _minPulseWidthUs = 544;
int _maxPulseWidthUs = 2400;
int _periodUs = 1000000 / DEFAULT_FREQUENCY;

float mapTemplate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int _angleToUs(float angle) {
    return mapTemplate(angle, _minAngle, _maxAngle, _minPulseWidthUs, _maxPulseWidthUs);
}

int _usToTicks(int us) { return std::round((PERIOD_TICKS * us) / _periodUs); }
int _ticksToUs(int duty) { return std::round((_periodUs * duty) / PERIOD_TICKS); }
float _usToAngle(int us) { return mapTemplate(us, _minPulseWidthUs, _maxPulseWidthUs, _minAngle, _maxAngle); }

void servoWriteMicroseconds(int pulseWidthUs) {
    pulseWidthUs = constrain(pulseWidthUs, _minPulseWidthUs, _maxPulseWidthUs);
    int _pulseWidthTicks = _usToTicks(pulseWidthUs);
    ledcWrite(SERVO_PWM_CHANNEL, _pulseWidthTicks);
}

namespace ServoHandler
{
    void init()
    {
        ledcSetup(SERVO_PWM_CHANNEL, DEFAULT_FREQUENCY, TIMER_RESOLUTION);
        ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
    }

    void servoWrite(float angle) {
        angle = constrain(angle, _minAngle, _maxAngle);
        servoWriteMicroseconds(_angleToUs(angle));
    }

    void servoUp()
    {
        servoWrite(SERVO_UP_POSITION);
    }

    void servoDown()
    {
        servoWrite(SERVO_DOWN_POSITION);
    }
}


