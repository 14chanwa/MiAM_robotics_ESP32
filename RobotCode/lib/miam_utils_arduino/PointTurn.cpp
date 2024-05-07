/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "PointTurn.h"
#include "Utilities.h"

#include <cmath>


namespace miam{
    namespace trajectory{
        PointTurn::PointTurn(TrajectoryConfig const& config,
                             RobotPosition const& startPoint,
                             float const& endAngle):
            Trajectory(config),
            endAngle_(endAngle)
        {
            description_ = "PointTurn";
            make(startPoint);
        }

        TrajectoryPoint PointTurn::getCurrentPoint(float const& currentTime)
        {
            TrajectoryPoint output;
            output.position = startPoint_;

            TrapezoidState state = trapezoid_.getState(currentTime);
            output.position.theta += motionSign_ * state.position;
            output.angularVelocity = motionSign_ * state.velocity;

            return output;
        }

        void PointTurn::make(RobotPosition const& startPoint)
        {
            startPoint_ = startPoint;
            motionSign_ = 1.0;
            // Create trapezoid.
            float length = moduloTwoPi(endAngle_ - startPoint.theta);
            if(length < 0)
                motionSign_ = -1.0;

            // Compute max angular velocity and acceleration, taking into account wheel spacing.
            float maxRobotAngularVelocity = config_.maxWheelVelocity / config_.robotWheelSpacing;
            float maxRobotAngularAcceleration = config_.maxWheelAcceleration / config_.robotWheelSpacing;

            trapezoid_ = Trapezoid(length, 0.0, 0.0, maxRobotAngularVelocity, maxRobotAngularAcceleration);
            duration_ = trapezoid_.getDuration();

        }


        void PointTurn::replanify(float const& replanificationTime, bool resetVelocity)
        {
            RobotPosition startPoint = getCurrentPoint(replanificationTime).position;
            // resetVelocity will have no effect since no linearVelocity
            make(startPoint);
        }

    }
}
