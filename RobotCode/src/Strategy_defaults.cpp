#include <Strategy.hpp>
#include <parameters.hpp>

#define SLOW_APPROACH_WHEEL_VELOCITY 75.0f
#define FINAL_TRAJECTORY_DISTANCE_CM 100.0f

namespace strategy
{
    float get_waiting_time_s()
    {
        #if PAMI_ID == 2
            return 3.0;
        #elif PAMI_ID == 5
            return 2.0;
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

    TrajectoryVector get_default_trajectory(MotionController* motionController)
    {
        RobotPosition startPosition;
        RobotPosition targetPosition;

        TrajectoryVector res;

        TrajectoryVector tv;
        std::vector<RobotPosition > positions;
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        
#if PAMI_ID == 1

        startPosition = RobotPosition(1230.0, 1925.0, -M_PI_2);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(80.0, 1450.0, M_PI);
        

        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.y -= 200;
        positions.push_back(tmp);
        tmp.x = 600;
        tmp.y = 1450.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

#elif PAMI_ID == 2

        startPosition = RobotPosition(1120.0, 1887.5, M_PI);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(750.0, 2000.0, M_PI_2);
        
        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = targetPosition;
        tmp.y = startPosition.y;
        tv = computeTrajectoryStraightLineToPoint(tc, startPosition, tmp);
        res.insert(res.end(), tv.begin(), tv.end());

        std::shared_ptr<Trajectory> pt(new PointTurn(tc, res.getEndPoint().position, M_PI_2));
        res.push_back(pt);

        tv = computeTrajectoryStraightLineToPoint(tc, res.getEndPoint().position, targetPosition);
        res.insert(res.end(), tv.begin(), tv.end());


#elif PAMI_ID == 3

        startPosition = RobotPosition(1120.0, 1962.5, M_PI);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(243.0, 1731.0, M_PI);

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

        startPosition = RobotPosition(1440.0, 1925.0, -M_PI_2);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(2635, 1145, 0);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.y -= 400;
        positions.push_back(tmp);
        tmp = targetPosition;
        tmp.x -= 200;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
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
        
#if PAMI_ID == 5

        // Option 2

        startPosition = RobotPosition(1440.0, 1925.0, -M_PI_2);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(2635, 1145, 0);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = RobotPosition(1437.0, 1728.0, 0.0);
        positions.push_back(tmp);
        tmp = RobotPosition(2120.0, 1487.0, 0.0);
        positions.push_back(tmp);
        tmp = targetPosition;
        tmp.x -= 200;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

#else
        res = strategy::get_default_trajectory(motionController);

#endif
        return res;
    }


    TrajectoryVector get_final_action_trajectory(MotionController* motionController)
    {
        // Go forward
        float distance = FINAL_TRAJECTORY_DISTANCE_CM;
        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        // Movement should be very slow
        tc.maxWheelVelocity = SLOW_APPROACH_WHEEL_VELOCITY;
        RobotPosition curPos(motionController->getCurrentPosition());
        return computeTrajectoryStraightLine(tc, curPos, distance);
    }
}
