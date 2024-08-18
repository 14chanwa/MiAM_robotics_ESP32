/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "PID.h"

namespace miam{
    PID::PID():
        Kp_(0.0),
        Kd_(0.0),
        Ki_(0.0),
        maxIntegral_(0.0),
        integral_(0.0),
        previousError_(0.0),
        lastCorrection_(0.0)
    {
    }


    PID::PID(float const& Kp, float const& Kd, float const& Ki, float const& maxIntegral):
        Kp_(Kp),
        Kd_(Kd),
        Ki_(Ki),
        maxIntegral_(maxIntegral),
        integral_(0.0),
        previousError_(0.0),
        lastCorrection_(0.0)
    {

    }


    float PID::computeValue(float const& error, float const& dt)
    {
        // Compute derivative.
        float derivative = 0.0;
        if(dt > 1e-6)
            derivative = (error - previousError_) / dt;

        return computeValue(error, derivative, dt);
    }


    float PID::computeValue(float const& error, float const& errorDerivative, float const& dt)
    {
        previousError_ = error;

        // Compute integral, clamped.
        integral_ = integral_ + dt * error;
        if(Ki_ > 1e-6)
        {
            if(Ki_ * integral_ > maxIntegral_)
                integral_ = maxIntegral_ / Ki_;
            if(Ki_ * integral_ < -maxIntegral_)
                integral_ = -maxIntegral_ / Ki_;
        }
        // Return result - minus because error is defined as current - target.
        lastCorrection_ = -Kp_ * (error + Kd_ * errorDerivative + Ki_ * integral_);
        return lastCorrection_;
    }


    void PID::resetIntegral(float const& value)
    {
        integral_ = value;
    }


    float PID::getCorrection()
    {
        return lastCorrection_;
    }


    float PID::getIntegral()
    {
        return integral_;
    }
}
