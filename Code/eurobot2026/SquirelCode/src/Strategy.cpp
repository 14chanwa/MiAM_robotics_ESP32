#include <Strategy.hpp>
#include <ServoHandler.hpp>
#include <Robot.hpp>
#include <RobotServos.hpp>
#include <cmath>
#include <I2CHandler.hpp>

/////////////////////////////////////////////////////////////////////
// Strategy
/////////////////////////////////////////////////////////////////////

TrajectoryVector traj;
#define PAMI_6_START 680.0, 1900.0, 0.0


#define SERVO_STEP_PER_TURN 4096
#define WHEEL_RADIUS_MM 30.0f
#define WHEEL_SPACING_MM (104.0f / 2.0f)

#define VLX_STOP_MM 150

int mm_to_tick(float mm)
{
    return mm * SERVO_STEP_PER_TURN / (2 * M_PI * WHEEL_RADIUS_MM);
}

int rad_to_tick(float rad)
{
    return rad * mm_to_tick(WHEEL_SPACING_MM);
}

byte servo_id_left = 1;
byte servo_id_right = 2;

void move_servo(int tick_left, int tick_right, bool translation)
{
    RobotServos::set_servo_position(servo_id_left, tick_left);
    RobotServos::set_servo_position(servo_id_right, tick_right);
    long time = millis();
    long waiting_time = 0;
    // if (translation)
    // {
    //     waiting_time = 1000 * abs(tick_left) / 1000.0; + 1800;
    // }
    // else
    // {
        //waiting_time = 800 * abs(tick_left) / 1000.0 + 1800;
    //}
    // Minimum waiting time
    waiting_time = 1000;
    while ((millis() - time < waiting_time) ||
        abs(RobotServos::get_current_speed(servo_id_left)) > 10 ||
        abs(RobotServos::get_current_speed(servo_id_right)) > 10
    )
    //while (RobotServos::is_moving(servo_id_left) || RobotServos::is_moving(servo_id_right))
    {
        uint16_t meas = I2CHandler::get_smoothed_vl53l0x();
        if (
            (!strategy::getHasObjectInSuction()) &&
            (meas < VLX_STOP_MM)
        )
        {
            RobotServos::stop(servo_id_left);
            RobotServos::stop(servo_id_right);
            while(true)
            {
                // freeze
                taskYIELD();
                delay(200);
            }
        }
        delay(200);

    }
}

void translate(float mm)
{
    move_servo(mm_to_tick(mm), -mm_to_tick(mm), true);
}

void rotate(float rad)
{
    if (Robot::getInstance()->motionController->isPlayingRightSide_)
    {
        rad = -rad;
    }
    move_servo(-rad_to_tick(rad), -rad_to_tick(rad), false);
}

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
        



        RobotServos::init_servo_id_step(servo_id_left);
        RobotServos::init_servo_id_step(servo_id_right);


        // Fetch first caisse
        // planner.go_forward(390);
        // planner.turn_around(-M_PI_2);
        // planner.execute();
        // RobotServos::set_servo_position(servo_id_left, -8000);
        // RobotServos::set_servo_position(servo_id_right, 8000);

        //delay(10000);

        //move_servo(8000, -8000);

        translate(390);
        // disable avoidance
        strategy::loadObjectInArm();

        rotate(-M_PI_2);

        // Recalage
        translate(-80);
        translate(70);


        ServoHandler::pumpOn();
        ServoHandler::armPositionMid();
        delay(500);
        ServoHandler::armPositionDown();
        delay(2000);
        ServoHandler::armPositionUpWithCrate();

        translate(-30);
        rotate(-M_PI_2);
        translate(420);


        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUpWithCrate();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUp();

        translate(-60);
        rotate(M_PI);

        // Recalage
        translate(-100);

        // 2d crate

        // activate avoidance
        strategy::freeObjectFromArm();
        translate(440);
        // disable avoidance
        strategy::loadObjectInArm();

        rotate(-M_PI_2);

        // Recalage
        translate(-100);
        translate(70);


        ServoHandler::pumpOn();
        ServoHandler::armPositionMid();
        delay(500);
        ServoHandler::armPositionDown();
        delay(2000);
        ServoHandler::armPositionUpWithCrate();

        rotate(-M_PI_2);
        translate(470);


        ServoHandler::pumpOff();
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUpWithCrate();
        delay(1000);
        ServoHandler::armPositionUpHorizontal();
        delay(500);
        ServoHandler::armPositionUp();

        translate(-60);
        rotate(M_PI);

        // Recalage
        translate(-100);

        // Go in front of zone

        // activate avoidance
        strategy::freeObjectFromArm();
        translate(520);
        // disable avoidance
        strategy::loadObjectInArm();


        rotate(-M_PI_2);

        // Recalage
        translate(-130);     
        
        // Go in front of zone
        translate(300);

        return;

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