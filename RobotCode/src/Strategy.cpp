#include <Strategy.hpp>
#include <Servo.hpp>

/////////////////////////////////////////////////////////////////////
// Strategy
/////////////////////////////////////////////////////////////////////

TrajectoryVector traj;

namespace strategy
{
    void go_forward(MotionController *motionController, float distance)
    {
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        RobotPosition curPos(motionController->getCurrentPosition());
        TrajectoryVector tv(computeTrajectoryStraightLine(tc, curPos, distance));
        motionController->setTrajectoryToFollow(tv);
        motionController->waitForTrajectoryFinished();
    }

    void turn_around(MotionController *motionController, float angle)
    {
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        traj.clear();
        RobotPosition curPos(motionController->getCurrentPosition());
        std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, curPos.theta + angle));
        traj.push_back(pt);
        motionController->setTrajectoryToFollow(traj);
        motionController->waitForTrajectoryFinished();
    }

    void go_to_point(MotionController *motionController, RobotPosition targetPoint)
    {
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        RobotPosition curPos(motionController->getCurrentPosition());
        TrajectoryVector tv(computeTrajectoryStraightLineToPoint(tc, curPos, targetPoint));
        motionController->setTrajectoryToFollow(tv);
        motionController->waitForTrajectoryFinished();
    }

    void make_a_square(MotionController *motionController)
    {
        RobotPosition startPosition(motionController->getCurrentPosition());
        RobotPosition targetPosition(startPosition);

        targetPosition.x += 1000;
        go_to_point(motionController, targetPosition);
        turn_around(motionController, M_PI_2);
        targetPosition.y += 1000;
        go_to_point(motionController, targetPosition);
        turn_around(motionController, M_PI_2);
        targetPosition.x -= 1000;
        go_to_point(motionController, targetPosition);
        turn_around(motionController, M_PI_2);
        targetPosition.y -= 1000;
        go_to_point(motionController, targetPosition);
        turn_around(motionController, M_PI_2);
    }

    void go_to_zone_3(MotionController *motionController)
    {
        motionController->resetPosition(RobotPosition(1275, 1925, -M_PI_2), true, true, true);
        RobotPosition startPosition(motionController->getCurrentPosition());
        RobotPosition targetPosition(startPosition);
        targetPosition.x = 2815;
        targetPosition.y = 980;

        go_to_point(motionController, targetPosition);
    }

    void perform_strategy(
        MotionController *motionController,
        float waiting_time_before_start_s, 
        TrajectoryVector trajectory_to_objective
    )
    {
        // Wait requested time
        vTaskDelay(waiting_time_before_start_s * 1000 /  portTICK_PERIOD_MS);

        // Execute move
        motionController->resetPosition(trajectory_to_objective.getCurrentPoint(0.0f).position, true, true, true);
        motionController->setTrajectoryToFollow(trajectory_to_objective);
        motionController->waitForTrajectoryFinished();

        // touch plant
        motionController->setSlowApproach(true);
        go_forward(motionController, 100.0);
        motionController->setSlowApproach(false);

        Servo::servoDown();
    }
}
