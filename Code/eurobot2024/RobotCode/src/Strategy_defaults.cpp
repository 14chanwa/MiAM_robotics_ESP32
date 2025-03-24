#include <Strategy.hpp>
#include <parameters.hpp>

#define SLOW_APPROACH_WHEEL_VELOCITY 75.0f
#define FINAL_TRAJECTORY_DISTANCE_MM 100.0f

namespace strategy
{
    float get_waiting_time_s()
    {
        #if PAMI_ID == 2
            return 1.5;
        #elif PAMI_ID == 5
            return 0.0;
        #else
            return 0.0;
        #endif
    };

    bool position_in_end_zone(RobotPosition position)
    {
        #if PAMI_ID == 1
        // Top left jardiniere
        return position.x <= 150 && position.y >= 1200 && position.y <= 1600; 
        #elif PAMI_ID == 3
        // top left blue corner
        return position.x <= 450 && position.y >= 1550;
        #elif PAMI_ID == 4
        // Bottom left blue corner
        return position.x <= 450 && position.y <= 450;
        #elif PAMI_ID == 5
        return position.x >= 2550 && position.y >= 760 && position.y <= 1200; 
        #else
        return false;
        #endif
    }

    bool position_in_avoidance_exclusion(RobotPosition position)
    {
        #if PAMI_ID == 3 || PAMI_ID == 4 || PAMI_ID == 5
        return position_in_end_zone(position);
        #else
        return false;
        #endif

        // #if PAMI_ID == 3
        // // top left blue corner
        // return position.x <= 600 && position.y >= 1400;
        // #elif PAMI_ID == 4
        // // Bottom left blue corner
        // return position.x <= 600 && position.y <= 600;
        // #elif PAMI_ID == 5
        // return position.x >= 2400 && position.y >= 650 && position.y <= 1350; 
        // #else
        // return false;
        // #endif
    }

    TrajectoryVector get_default_trajectory(MotionController* motionController)
    {
        RobotPosition startPosition;
        RobotPosition targetPosition;

        TrajectoryVector res;

        TrajectoryVector tv;
        std::vector<RobotPosition > positions;
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        
#if PAMI_ID == 1

        startPosition = RobotPosition(35.0, 1600.0, 0);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(1900.0, 1420.0, M_PI);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.x += 200;
        positions.push_back(tmp);
        tmp.x = 550;
        tmp.y = 1500.0;
        positions.push_back(tmp);
        tmp.x = 1450.0;
        tmp.y = 1096.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

#elif PAMI_ID == 2

        startPosition = RobotPosition(35.0, 1700.0, 0);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(1500.0, 1400.0, M_PI);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.x += 150;
        positions.push_back(tmp);
        tmp.x = 1000.0;
        tmp.y = 1300.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());


#elif PAMI_ID == 3

        startPosition = RobotPosition(1120.0, 1962.5, M_PI);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(243.0, 1681.0, M_PI);

        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.x -= 400;
        positions.push_back(tmp);
        tmp = targetPosition;
        tmp.x += 400;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

#elif PAMI_ID == 4

        startPosition = RobotPosition(1330.0, 1925.0, -M_PI_2);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(225, 225, -M_PI_2-M_PI_4);
        

        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.y -= 400;
        positions.push_back(tmp);
        tmp = RobotPosition(1140.0, 1048.0, 0.0);
        positions.push_back(tmp);
        tmp = RobotPosition(615.0, 598.0, 0.0);
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

#elif PAMI_ID == 5

        // Option 1

        startPosition = RobotPosition(35.0, 1905.0, 0.0);
        motionController->resetPosition(startPosition, true, true, true);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = startPosition;
        tmp.x += 1300.0;
        positions.push_back(tmp);
        tmp.x -= 150;
        tmp.y -= 350;
        positions.push_back(tmp);

        // // straight line towards bottom
        // tmp = startPosition;
        // tmp.y -= 400;
        // positions.push_back(tmp);
        // tmp = targetPosition;
        // tmp.x -= 200;
        // positions.push_back(tmp);
        // positions.push_back(targetPosition);
        tv = computeTrajectoryRoundedCorner(tc, positions, 100.0);
        res.insert(res.end(), tv.begin(), tv.end());

#else

        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        RobotPosition curPos(motionController->getCurrentPosition());
        TrajectoryVector tv(computeTrajectoryStraightLine(tc, curPos, 300.0));
        res.insert(res.end(), tv.begin(), tv.end());

#endif
    return res;

    };

    TrajectoryVector get_alternative_trajectory(MotionController* motionController)
    {
        RobotPosition startPosition;
        RobotPosition targetPosition;

        TrajectoryVector res;

        TrajectoryVector tv;
        std::vector<RobotPosition > positions;
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        res = strategy::get_default_trajectory(motionController);
        return res;
    }


    TrajectoryVector get_final_action_trajectory(MotionController* motionController)
    {
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        TrajectoryVector res;
//         RobotPosition currentPosition(motionController->getCurrentPosition());
// #if PAMI_ID == 1
//         std::shared_ptr<Trajectory > pointTurnTowardsLeft(
//         std::make_shared<PointTurn >(
//                 tc, 
//                 currentPosition, 
//                 M_PI)
//         );
//         res.push_back(pointTurnTowardsLeft);
//         currentPosition = res.back()->getEndPoint().position;
// #endif
//         // Go forward
//         float distance = FINAL_TRAJECTORY_DISTANCE_MM;
//         // Movement should be very slow
//         tc.maxWheelVelocity = SLOW_APPROACH_WHEEL_VELOCITY;
//         TrajectoryVector sl = computeTrajectoryStraightLine(tc, currentPosition, distance);
//         res.insert(res.end(), sl.begin(), sl.end());

//         for (auto traj : res)
//         {
//                 traj->setAvoidanceEnabled(false);
//         }
        return res;
    }
}
