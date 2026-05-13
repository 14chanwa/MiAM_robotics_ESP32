#include <Strategy.hpp>
#include <ServoHandler.hpp>
#include <Robot.hpp>
#include <cmath>

/////////////////////////////////////////////////////////////////////
// Strategy
/////////////////////////////////////////////////////////////////////

TrajectoryVector traj;
#define PAMI_6_START 680.0, 1900.0, 0.0

namespace strategy
{
    bool hasObjectInSuction_ = false;
    
    bool getHasObjectInSuction() { return hasObjectInSuction_; };
    void loadObjectInArm() { hasObjectInSuction_ = true; }
    void freeObjectFromArm() { hasObjectInSuction_ = false; }


    void perform_strategy()
    {
        StrategyPlanner planner;

        Robot* robot = Robot::getInstance();
        MotionController* motionController = robot->motionController;

        RobotPosition startPosition;
        RobotPosition targetPosition;

        TrajectoryVector res;

        TrajectoryVector tv;
        std::vector<RobotPosition > positions;


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
        
        strategy::freeObjectFromArm();

        // Fetch first caisse
        planner.go_forward(390);
        planner.turn_around(-M_PI_2);
        planner.execute();

        // Recalage
        planner.go_forward(-60);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 1950;
        targetPosition.theta = -M_PI_2;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);

        // Fetch first caisse
        planner.go_forward(70);
        planner.execute();

        ServoHandler::pumpOn();
        ServoHandler::armPositionMid();
        delay(500);
        ServoHandler::armPositionDown();
        delay(2000);
        ServoHandler::armPositionUpWithCrate();
        strategy::loadObjectInArm();

        planner.go_forward(-20);
        planner.execute();

        planner.turn_around(-M_PI / 2.1); // Make it turn counter clockwise
        planner.turn_to_angle(M_PI);

        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 636;
        planner.go_to_point(targetPosition);
        planner.execute();

        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUpWithCrate();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        strategy::freeObjectFromArm();

        // Recalage sur bordure verticale : déjà contre la bordure
        planner.go_forward(30);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 690;
        targetPosition.theta = M_PI;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);
        planner.go_forward(-60);

        // Recalage sur bordure horizontale
        planner.turn_to_angle( -M_PI_2);
        planner.go_forward(-150);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 1950;
        targetPosition.theta = -M_PI_2;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);
        planner.go_forward(60);

        // Aller chercher la 2e caisse
        planner.turn_to_angle(0);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 1130;
        planner.go_to_point(targetPosition);
        planner.turn_to_angle(-M_PI / 2);
        planner.execute();

        // Recalage
        planner.go_forward(-60);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 1950;
        targetPosition.theta = -M_PI_2;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);

        planner.go_forward(80);
        planner.execute();

        ServoHandler::pumpOn();
        ServoHandler::armPositionDown();
        delay(2000);
        ServoHandler::armPositionUpWithCrate();
        strategy::loadObjectInArm();

        planner.go_forward(-20);
        planner.turn_around(-M_PI / 2.1); // Make it turn counter clockwise
        planner.turn_to_angle(M_PI);
        planner.go_to_point(RobotPosition(636, 1907, 0));
        planner.execute();

        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUp();
        delay(500);
        ServoHandler::armPositionDown();
        delay(500);
        ServoHandler::armPositionUp();
        strategy::freeObjectFromArm();

        // Recalage sur bordure verticale : déjà contre la bordure
        planner.go_forward(30);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 690;
        targetPosition.theta = M_PI;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);
        planner.go_forward(-60);

        planner.turn_to_angle(-M_PI_2);
        planner.execute();

        // Recalage
        planner.go_forward(-130);
        planner.execute();
        targetPosition = motionController->getCurrentPosition();
        targetPosition.x = 615;
        targetPosition.theta = 0;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);

        return;

        // Virer les caisses noires
        strategy::go_forward(motionController, 10);
        strategy::turn_to_angle(motionController, -3* M_PI / 2);
        strategy::go_forward(motionController, 50);

        ServoHandler::armPositionMid();
        strategy::turn_around(motionController, M_PI_2);
        strategy::turn_around(motionController, -M_PI_4);
        delay(500);
        ServoHandler::armPositionUp();
        delay(500);

        // Recalage
        strategy::go_forward(motionController, -150);
        targetPosition = motionController->getCurrentPosition();
        targetPosition.y = 615;
        targetPosition.theta = M_PI;
        delay(500);
        motionController->resetPosition(targetPosition, true, true, true);

        strategy::go_forward(motionController, 115);
        //strategy::turn_around(motionController, -M_PI / 8);

        ServoHandler::pumpOn();
        ServoHandler::armPositionDown();
        delay(2000);
        ServoHandler::armPositionUp();

        strategy::go_forward(motionController, -100);
        strategy::turn_to_angle(motionController, M_PI);

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

    void turn_to_angle(MotionController *motionController, float end_angle)
    {
        Serial.println("TurnAround");
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        traj.clear();
        RobotPosition curPos(motionController->getCurrentPosition());
        std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, end_angle));
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


StrategyPlanner::StrategyPlanner()
{
    motionController = Robot::getInstance()->motionController;
    tc = motionController->getTrajectoryConfig();
}

void StrategyPlanner::add_to_trajectory(TrajectoryVector tv)
{
    for (auto it = std::begin(tv); it != std::end(tv); it++) {
        trajectory_.push_back(*it);
    }
}

RobotPosition StrategyPlanner::get_last_position()
{
    RobotPosition curPos(motionController->getCurrentPosition());
    if (!trajectory_.empty())
    {
        curPos = trajectory_.getEndPoint().position;
    }
    return curPos;
}

void StrategyPlanner::go_forward(float distance)
{
    RobotPosition curPos = get_last_position();
    TrajectoryVector tv(computeTrajectoryStraightLine(tc, curPos, distance));
    add_to_trajectory(tv);
}

void StrategyPlanner::turn_around(float angle)
{
    RobotPosition curPos = get_last_position();
    std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, curPos.theta + angle));
    trajectory_.push_back(pt);
}

void StrategyPlanner::turn_to_angle(float angle)
{
    RobotPosition curPos = get_last_position();
    std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, angle));
    trajectory_.push_back(pt);
}

void StrategyPlanner::execute()
{
    motionController->setTrajectoryToFollow(trajectory_);
    motionController->waitForTrajectoryFinished();
    clear();
}

void StrategyPlanner::go_to_point(RobotPosition targetPoint)
{
    RobotPosition curPos = get_last_position();
    TrajectoryVector tv(computeTrajectoryStraightLineToPoint(tc, curPos, targetPoint));
    add_to_trajectory(tv);
}