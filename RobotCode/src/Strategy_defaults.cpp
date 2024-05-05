#include <Strategy.hpp>
#include <parameters.hpp>

namespace strategy
{
    float get_waiting_time_s()
    {
        #if PAMI_ID == 2
            return 3.0;
        #else
            return 0.0;
        #endif
    };

    bool position_in_end_zone(RobotPosition position)
    {
        #if PAMI_ID == 4
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
        targetPosition = RobotPosition(80.0, 1375.0, M_PI);
        

        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.y -= 300;
        positions.push_back(tmp);
        tmp = targetPosition;
        tmp.x += 50.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 100.0);
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
        targetPosition = RobotPosition(243.0, 1781.0, M_PI);

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

        startPosition = RobotPosition(1440.0, 1925.0, -M_PI_2);
        motionController->resetPosition(startPosition, true, true, true);
        targetPosition = RobotPosition(2700, 950, 0);
        
        positions.clear();
        positions.push_back(startPosition);

        RobotPosition tmp = targetPosition;

        // straight line towards bottom
        tmp = startPosition;
        tmp.y -= 400;
        positions.push_back(tmp);
        tmp = RobotPosition(1597.0, 1013.0, 0.0);
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
}
