/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <math.h>
#include <L6470Driver.h>

// Dimensions of the robot
namespace robotdimensions
{
    float const wheelRadius = 12.5;          ///< Wheel radius, in mm - identified during open loop experiments.
    float const wheelSpacing = 30.0;        ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
    float const encoderWheelRadius = 25.3;   ///< Radius of encoder wheels, in mm.
    float const encoderWheelSpacing = 139.0; ///< Encoder wheel spacing from robot center, in mm.

    float const stepSize = 2 * M_PI / 600.0; ///< Size of a motor step, in rad.

    float const maxWheelSpeed = 600;         ///< Maximum wheel speed, in mm/s.
    float const maxWheelAcceleration = 800; ///< Maximum wheel acceleration, in mm/s^2.

    miam::L6470_STEP_MODE const stepMode = miam::MICRO_4;
}

#define MIAM_SCD_ADDRESS IPAddress(42, 42, 0, 10)
#define MIAM_SCD_PORT 778

#define USE_DC_MOTORS

#endif