#ifndef _ROBOTPARAMETERS_H
#define _ROBOTPARAMETERS_H

// conversion functions
#define RPM_TO_RAD_S(VALUE) (VALUE * 2.0f * M_PI / 60.0f)
#define RAD_S_TO_RPM(VALUE) (VALUE * 60.0f / (2.0f * M_PI))

struct RobotParameters
{
    float maxWheelSpeed;
    float maxWheelAcceleration;
    float wheelRadius;
    float wheelSpacing;
};

#endif