/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "Trapezoid.h"

#include <cmath>

// Crop value to be between -max and max
float crop(float value, float min, float max)
{
    if(value < min)
        return min;
    if(value > max)
        return max;
    return value;
}

namespace miam{
    namespace trajectory{
        Trapezoid::Trapezoid():
            duration_(0.0),
            maxVelocity_(0.0),
            maxAcceleration_(0.0),
            startVelocity_(0.0),
            length_(0.0),
            timeToStopAccelerating_(0.0),
            timeToStartDecelerating_(0.0),
            accelerationDistance_(0.0)
        {
        }


        Trapezoid::Trapezoid(float const& distance,
                             float const& startVelocity,
                             float const& endVelocity,
                             float maxVelocity,
                             float maxAcceleration):
            maxVelocity_(std::abs(maxVelocity)),
            maxAcceleration_(std::abs(maxAcceleration)),
            length_(std::abs(distance))
        {
            // Crop start and end velocity to the maximum values.
            float vi = crop(startVelocity, -maxVelocity_, maxVelocity_);
            float ve = crop(endVelocity, 0.0, maxVelocity_);
            startVelocity_ = vi;

            // Compute, with given constraints, minimum trajectory length to reach a constant velocity phase.
            accelerationDistance_ = (maxVelocity_ * maxVelocity_ - vi * vi) / maxAcceleration_ / 2.0;
            float decelerationDistance = (maxVelocity_ * maxVelocity_ - ve * ve) / maxAcceleration_ / 2.0;

            if(length_ > accelerationDistance_ + decelerationDistance)
            {
                // We have a constant velocity phase, compute time flags accordingly.
                timeToStopAccelerating_ = (maxVelocity_ - vi) / maxAcceleration_;
                timeToStartDecelerating_ = timeToStopAccelerating_ +
                                          (length_ - (accelerationDistance_ + decelerationDistance)) / maxVelocity_;
                // Compute total duration.
                duration_ = timeToStartDecelerating_ + (maxVelocity_ - ve) / maxAcceleration_;
            }
            else
            {
                // We have no constant velocity phase: look to see if distance is long enough to reach the target.
                // Get the time at which we have to stop accelerating, which will also be the time at which we have
                // to start deccelerating.
                float discriminent = 4 * maxAcceleration_ * length_ + 2 * vi * vi + 2 * ve * ve;
                float stopAccelTime = (std::sqrt(discriminent) - 2.0 * vi) / (2.0 * maxAcceleration_);

                // Clamp it between 0 and the maximum duration, i.e. the time it would take with maximum acceleration
                // to reach the target point.
                float maxDuration = (- vi + std::sqrt(vi * vi + 2 * maxAcceleration_ * length_)) / maxAcceleration_;
                timeToStopAccelerating_ = crop(stopAccelTime, 0.0, maxDuration);
                timeToStartDecelerating_ = timeToStopAccelerating_;

                // Update maxVelocity to match the real maximum velocity of the trajectory.
                maxVelocity_ = vi + maxAcceleration_ * timeToStopAccelerating_;

                // Compute acceleration distance and duration accordingly.
                accelerationDistance_ = (maxVelocity_ * maxVelocity_  - vi * vi) / maxAcceleration_ / 2.0;
                // Duration will depend on whether we can reach the target final velocity.
                if(timeToStopAccelerating_ == stopAccelTime)
                {
                    // Normal case, just compute the time taking into account the final velocity.
                    duration_ = timeToStopAccelerating_ + (maxVelocity_ - ve) / maxAcceleration_;
                }
                else
                {
                    // We can't reach terminal velocity, so we will be constantly accelerating / decelerating.
                    if(vi > ve)
                    {
                        // Constant acceleration, i.e. maxDuration.
                        duration_ = maxDuration;
                    }
                    else
                    {
                        // Constant deceleration
                        duration_ = (-vi + std::sqrt(vi * vi + 2 * maxAcceleration_ * length_)) / maxAcceleration_;
                    }
                }
            }
        }

        TrapezoidState Trapezoid::getState(float const& currentTime)
        {
            TrapezoidState output;
            if(currentTime < 0.0)
                return output;
            if(currentTime >= duration_)
            {
                output.position = length_;
                return output;
            }
            else if(currentTime < timeToStopAccelerating_)
            {
                output.velocity = startVelocity_ + maxAcceleration_ * currentTime;
                output.position = startVelocity_ * currentTime + maxAcceleration_ / 2.0 * currentTime * currentTime;
            }
            else if(currentTime < timeToStartDecelerating_)
            {
                output.velocity = maxVelocity_;
                output.position = accelerationDistance_ + maxVelocity_ * (currentTime - timeToStopAccelerating_);
            }
            else
            {
                output.velocity = maxVelocity_ - maxAcceleration_ * (currentTime - timeToStartDecelerating_);
                output.position = accelerationDistance_ +
                                           maxVelocity_ * (timeToStartDecelerating_ - timeToStopAccelerating_)
                                           + maxVelocity_* (currentTime - timeToStartDecelerating_)
                                           - maxAcceleration_ / 2.0 * (currentTime - timeToStartDecelerating_) * (currentTime - timeToStartDecelerating_);
            }
            return output;
        }

        float Trapezoid::getDuration()
        {
            return duration_;
        }
    }
}
