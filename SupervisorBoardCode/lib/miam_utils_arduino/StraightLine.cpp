/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "StraightLine.h"
#include "Utilities.h"

#include <cmath>

namespace miam{
    namespace trajectory{
        StraightLine::StraightLine(TrajectoryConfig const& config,
                                   RobotPosition const& startPoint,
                                   RobotPosition const& endPoint,
                                   float const& startVelocity,
                                   float const& endVelocity,
                                   bool const& backward):
            Trajectory(config),
            endPoint_(endPoint),
            endVelocity_(endVelocity),
            backward_(backward)
        {
            description_ = "StraightLine";
            make(startPoint, startVelocity);
        }


        void StraightLine::make(RobotPosition const& startPoint, float const& startVelocity)
        {
            startPoint_ = startPoint;
            motionSign_ = 1.0;

            if(backward_)
                motionSign_ = -1.0;
            // Create trapezoid.
            float length = distance(startPoint, endPoint_);
            trapezoid_ = Trapezoid(length, startVelocity, endVelocity_, config_.maxWheelVelocity, config_.maxWheelAcceleration);

            duration_ = trapezoid_.getDuration();

            // Compute angle.
            startPoint_.theta += computeShortestAngle(startPoint, endPoint_);

            if(backward_)
            {
                // Add or remove pi if going backward.
                if(startPoint_.theta < 0)
                    startPoint_.theta += M_PI;
                else
                    startPoint_.theta -= M_PI;
            }
        }

        TrajectoryPoint StraightLine::getCurrentPoint(float const& currentTime)
        {
            TrajectoryPoint output;
            output.position = startPoint_;
            TrapezoidState state = trapezoid_.getState(currentTime);

            output.linearVelocity = motionSign_ * state.velocity;

            // Compute position.
            output.position.x += motionSign_ * state.position * std::cos(startPoint_.theta);
            output.position.y += motionSign_ * state.position * std::sin(startPoint_.theta);

            return output;
        }


        void StraightLine::replanify(float const& replanificationTime)
        {
            RobotPosition startPoint = getCurrentPoint(replanificationTime).position;
            make(startPoint, 0.0);
        }


        float StraightLine::getAngle()
        {
            return startPoint_.theta;
        }
    }
}
