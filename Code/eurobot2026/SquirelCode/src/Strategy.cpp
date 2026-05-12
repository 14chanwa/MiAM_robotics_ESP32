#include <Strategy.hpp>
#include <ServoHandler.hpp>
#include <Robot.hpp>
#include <cmath>

/////////////////////////////////////////////////////////////////////
// Strategy
/////////////////////////////////////////////////////////////////////

TrajectoryVector traj;
#define PAMI_6_START 680.0, 1860.0, 0.0

namespace strategy
{
    void perform_strategy()
    {
        Robot* robot = Robot::getInstance();
        MotionController* motionController = robot->motionController;
        RobotPosition startPosition;
        RobotPosition targetPosition;

        TrajectoryVector res;

        TrajectoryVector tv;
        std::vector<RobotPosition > positions;
        TrajectoryConfig tc = motionController->getTrajectoryConfig();

        startPosition = RobotPosition(PAMI_6_START);
        ServoHandler::armPositionUp();

        motionController->resetPosition(startPosition, true, true, true);
        positions.clear();
        // positions.push_back(startPosition);
        // positions.push_back(RobotPosition(895, 1926, 0));
        // positions.push_back(RobotPosition(992, 1850, 0));
        // positions.push_back(RobotPosition(1003, 1805, 0));

        // tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        // res.insert(res.end(), tv.begin(), tv.end());

        // motionController->setTrajectoryToFollow(tv);
        // motionController->waitForTrajectoryFinished();

        // Fetch first caisse
        strategy::go_forward(motionController, 250);
        strategy::turn_around(motionController, M_PI / 6);
        strategy::go_forward(motionController, 10);

        ServoHandler::pumpOn();
        ServoHandler::armPositionDown();
        delay(3000);
        ServoHandler::armPositionUp();

        strategy::go_forward(motionController, -50);
        strategy::turn_around(motionController, -M_PI / 2.1); // Make it turn counter clockwise
        strategy::turn_to_angle(motionController, M_PI);

        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 636;
        strategy::go_to_point(motionController, targetPosition);

        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionDown();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        strategy::go_forward(motionController, -100);
        strategy::turn_to_angle(motionController, 0);

        // Recalage
        strategy::turn_to_angle(motionController, 0);
        strategy::go_forward(motionController, -130);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 660;
        targetPosition.theta = 0;
        delay(500);
        motionController->setCurrentPosition(targetPosition);

        strategy::go_forward(motionController, 100);
        strategy::turn_to_angle(motionController, -M_PI_2);
        strategy::go_forward(motionController, -130);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 1950;
        targetPosition.theta = -M_PI_2;
        delay(500);
        motionController->setCurrentPosition(targetPosition);
        strategy::go_forward(motionController, 100);

        // Aller chercher la 2e caisse
        strategy::turn_to_angle(motionController, 0);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x += 275;
        strategy::go_to_point(motionController, targetPosition);
        strategy::turn_around(motionController, M_PI / 5);
        strategy::go_forward(motionController, 50);

        ServoHandler::pumpOn();
        ServoHandler::armPositionDown();
        delay(3000);
        ServoHandler::armPositionUp();

        strategy::go_forward(motionController, -100);
        strategy::turn_around(motionController, -M_PI / 2.1); // Make it turn counter clockwise
        strategy::turn_to_angle(motionController, M_PI);

        tv = computeTrajectoryStraightLineToPoint(tc, motionController->getCurrentPosition(), RobotPosition(636, 1907, 0));
        res.insert(res.end(), tv.begin(), tv.end());

        motionController->setTrajectoryToFollow(tv);
        motionController->waitForTrajectoryFinished();

        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionDown();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        strategy::go_forward(motionController, -100);
        strategy::turn_to_angle(motionController, 0);

        // Recalage
        strategy::go_forward(motionController, -130);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 615;
        targetPosition.theta = 0;
        delay(500);
        motionController->setCurrentPosition(targetPosition);

        // Virer les caisses noires
        strategy::go_forward(motionController, 10);
        strategy::turn_to_angle(motionController, -3* M_PI / 2);
        strategy::go_forward(motionController, 50);

        ServoHandler::armPositionMid();
        strategy::turn_around(motionController, -M_PI_2);
        strategy::turn_around(motionController, M_PI_4);
        delay(500);
        ServoHandler::armPositionUp();
        delay(500);

        // Recalage
        strategy::go_forward(motionController, -150);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 615;
        targetPosition.theta = M_PI;
        delay(500);
        motionController->setCurrentPosition(targetPosition);

        strategy::go_forward(motionController, 115);
        //strategy::turn_around(motionController, -M_PI / 8);

        ServoHandler::pumpOn();
        ServoHandler::armPositionDown();
        delay(3000);
        ServoHandler::armPositionUp();

        strategy::go_forward(motionController, -100);
        strategy::turn_around(motionController, M_PI_2);

        strategy::go_forward(motionController, 50);

        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionDown();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        strategy::go_forward(motionController, -100);
        strategy::turn_to_angle(motionController, 0);
    }

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
        Serial.println("TurnAround");
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        traj.clear();
        RobotPosition curPos(motionController->getCurrentPosition());
        std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, curPos.theta + angle));
        traj.push_back(pt);
        motionController->setTrajectoryToFollow(traj);
        motionController->waitForTrajectoryFinished();
    }

    void turn_to_angle(MotionController *motionController, float angle)
    {
        Serial.println("TurnAround");
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        traj.clear();
        RobotPosition curPos(motionController->getCurrentPosition());
        std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, angle));
        traj.push_back(pt);
        motionController->setTrajectoryToFollow(traj);
        motionController->waitForTrajectoryFinished();
    }

    void go_to_point(MotionController *motionController, RobotPosition targetPoint)
    {
        Serial.println("GoToPoint");
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
        TrajectoryConfig tc = motionController->getTrajectoryConfig();

        TrajectoryVector tv;
        TrajectoryVector tv2;

        targetPosition.x += 500;
        tv2 = computeTrajectoryStraightLineToPoint(tc, startPosition, targetPosition);
        tv.insert(tv.end(), tv2.begin(), tv2.end());

        {
            startPosition = targetPosition;
            targetPosition.theta += M_PI_2;
            std::shared_ptr<Trajectory> pt(new PointTurn(tc, startPosition, targetPosition.theta));
            tv.push_back(pt);
        }

        startPosition = targetPosition;
        targetPosition.y += 500;
        tv2 = computeTrajectoryStraightLineToPoint(tc, startPosition, targetPosition);
        tv.insert(tv.end(), tv2.begin(), tv2.end());

        {
            startPosition = targetPosition;
            targetPosition.theta += M_PI_2;
            std::shared_ptr<Trajectory> pt(new PointTurn(tc, startPosition, targetPosition.theta));
            tv.push_back(pt);
        }

        startPosition = targetPosition;
        targetPosition.x -= 500;
        tv2 = computeTrajectoryStraightLineToPoint(tc, startPosition, targetPosition);
        tv.insert(tv.end(), tv2.begin(), tv2.end());

        {
            startPosition = targetPosition;
            targetPosition.theta += M_PI_2;
            std::shared_ptr<Trajectory> pt(new PointTurn(tc, startPosition, targetPosition.theta));
            tv.push_back(pt);
        }

        startPosition = targetPosition;
        targetPosition.y -= 500;
        tv2 = computeTrajectoryStraightLineToPoint(tc, startPosition, targetPosition);
        tv.insert(tv.end(), tv2.begin(), tv2.end());

        {
            startPosition = targetPosition;
            targetPosition.theta += M_PI_2;
            std::shared_ptr<Trajectory> pt(new PointTurn(tc, startPosition, targetPosition.theta));
            tv.push_back(pt);
        }
        motionController->setTrajectoryToFollow(tv);
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

    // void perform_strategy_travel_to_objective(
    //     MotionController *motionController,
    //     float waiting_time_before_start_s, 
    //     TrajectoryVector trajectory_to_objective
    // )
    // {
    //     // Wait requested time
    //     vTaskDelay(waiting_time_before_start_s * 1000 /  portTICK_PERIOD_MS);

    //     // Execute move
    //     motionController->resetPosition(trajectory_to_objective.getCurrentPoint(0.0f).position, true, true, true);
    //     motionController->setTrajectoryToFollow(trajectory_to_objective);
    // }

    // void perform_strategy_final_approach(
    //     MotionController *motionController,
    //     float waiting_time_before_start_s, 
    //     TrajectoryVector trajectory_to_objective
    // )
    // {
    //     // touch plant
    //     motionController->setSlowApproach(true);
    //     go_forward(motionController, 100.0);
    //     motionController->setSlowApproach(false);

    //     ServoHandler::servoDown();
    // }
}
