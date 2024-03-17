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

        return res;

#elif PAMI_ID == 2

        startPosition = RobotPosition(1120.0, 1887.5, M_PI);

#elif PAMI_ID == 3

        startPosition = RobotPosition(1120.0, 1962.5, M_PI);

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

        return res;

#elif PAMI_ID == 5

        startPosition = RobotPosition(1440.0, 1925.0, -M_PI_2);

#else

        TrajectoryConfig tc = motionController->getTrajectoryConfig();
        RobotPosition curPos(motionController->getCurrentPosition());
        TrajectoryVector tv(computeTrajectoryStraightLine(tc, curPos, 300.0));
        return tv;

#endif

    };
}
