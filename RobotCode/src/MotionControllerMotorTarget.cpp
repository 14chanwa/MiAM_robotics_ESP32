#include <MotionController.hpp>
#include <parameters.hpp>

bool MotionController::computeMotorTarget(Trajectory *traj,
                                          float const &timeInTrajectory,
                                          float const &dt,
                                          float const &slowDownRatio,
                                          DrivetrainMeasurements const &measurements,
                                          DrivetrainTarget &target)
{
    // Get current trajectory state.
    targetPoint = traj->getCurrentPoint(curvilinearAbscissa_);

    // Update trajectory velocity based on lidar coeff.
    targetPoint.linearVelocity *= slowDownRatio;
    targetPoint.angularVelocity *= slowDownRatio;

    // Compute targets for rotation and translation motors.
    BaseSpeed targetSpeed;

    // Feedforward.
    targetSpeed.linear = targetPoint.linearVelocity;
    targetSpeed.angular = targetPoint.angularVelocity;

    // Compute error.
    RobotPosition currentPosition = currentPosition_;
    RobotPosition error = currentPosition - targetPoint.position;

    // Rotate by -theta to express the error in the tangent frame.
    RobotPosition rotatedError = error.rotate(-targetPoint.position.theta);

    float trackingLongitudinalError = rotatedError.x;
    float trackingTransverseError = rotatedError.y;

    // Change sign if going backward.
    if (targetPoint.linearVelocity < 0)
        trackingTransverseError = -trackingTransverseError;

    // std::cout << "start trackingAngleError" << std::endl;
    // std::cout << "currentPosition " << currentPosition << std::endl;
    // std::cout << "targetPoint " << targetPoint << std::endl;
    float trackingAngleError = miam::trajectory::moduloTwoPi(currentPosition.theta - targetPoint.position.theta);
    // std::cout << "end trackingAngleError" << std::endl;

    // If we are beyond trajectory end, look to see if we are close enough to the target point to stop.
    if (traj->getDuration() <= curvilinearAbscissa_)
    {
        if (trackingLongitudinalError < 2 && trackingAngleError < 0.01 && measurements.motorSpeed[side::RIGHT] < 0.5 && measurements.motorSpeed[side::RIGHT] < 0.5)
        {
            // Just stop the robot.
            target.motorSpeed[0] = 0.0;
            target.motorSpeed[1] = 0.0;
            return true;
        }
    }

    // Compute correction terms.

    // If trajectory has an angular velocity but no linear velocity, it's a point turn:
    // disable corresponding position servoing.
    if (!(std::abs(targetPoint.angularVelocity) > 0.1 && std::abs(targetPoint.linearVelocity) < 0.1))
        targetSpeed.linear += PIDLinear_.computeValue(trackingLongitudinalError, dt);

    // Modify angular PID target based on transverse error, if we are going fast enough.
    float angularPIDError = trackingAngleError;
    float transverseCorrection = 0.0;
    if (std::abs(targetPoint.linearVelocity) > 0.1 * parameters_.maxWheelSpeed)
        transverseCorrection = TRANSVERSE_KP * targetPoint.linearVelocity / parameters_.maxWheelSpeed * trackingTransverseError;
    if (targetPoint.linearVelocity < 0)
        transverseCorrection = -transverseCorrection;
    angularPIDError += transverseCorrection;

    targetSpeed.angular += PIDAngular_.computeValue(angularPIDError, dt);

    // Invert velocity if playing on side::RIGHT side.
    if (isPlayingRightSide_)
        targetSpeed.angular = -targetSpeed.angular;

    // save target
    targetSpeed_ = targetSpeed;

    // Convert from base velocity to motor wheel velocity.
    WheelSpeed wheelSpeed = kinematics_.inverseKinematics(targetSpeed);
    // Convert to motor unit.
    target.motorSpeed[side::RIGHT] = wheelSpeed.right;
    target.motorSpeed[side::LEFT] = wheelSpeed.left;

    // Clamp to maximum speed
    float const maxAngularVelocity = parameters_.maxWheelSpeed / parameters_.wheelRadius;
    for (int i = 0; i < 2; i++)
    {
        if (target.motorSpeed[i] > maxAngularVelocity)
            target.motorSpeed[i] = maxAngularVelocity;
        if (target.motorSpeed[i] < -maxAngularVelocity)
            target.motorSpeed[i] = -maxAngularVelocity;
    }

    return false;
}