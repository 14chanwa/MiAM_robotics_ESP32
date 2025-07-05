#include <Strategy.hpp>
#include <parameters.hpp>

#define SLOW_APPROACH_WHEEL_VELOCITY 75.0f
#define FINAL_TRAJECTORY_DISTANCE_MM 100.0f

// xmin xmax ymin ymax
#define COORD_ZONE_1 960.0, 1250.0, 1380.0, 1540.0
#define COORD_ZONE_2 1300.0, 1750.0, 1300.0, 1540.0
#define COORD_ZONE_3 1740.0, 2000.0, 1380.0, 1540.0

#define PAMI_1_START 45.0, 1585.0, 0.0
#define PAMI_2_START 45.0, 1665.0, 0.0
#define PAMI_3_START 45.0, 1745.0, 0.0
#define PAMI_4_START 45.0, 1825.0, 0.0
#define PAMI_5_START 35.0, 1905.0, 0.0

#define PAMI_1_WAIT 6.0
#define PAMI_2_WAIT 4.0
#define PAMI_3_WAIT 0.0
#define PAMI_4_WAIT 1.0
#define PAMI_5_WAIT 0.0

bool robot_position_in_zone(RobotPosition& position, float xmin, float xmax, float ymin, float ymax)
{
    return (
        position.x >= xmin &&
        position.x <= xmax &&
        position.y >= ymin &&
        position.y <= ymax
    );
}

namespace strategy
{
    float get_waiting_time_s()
    {
        #if PAMI_ID == 1
            return PAMI_1_WAIT;
        #elif PAMI_ID == 2
            return PAMI_2_WAIT;
        #elif PAMI_ID == 3
            return PAMI_3_WAIT;
        #elif PAMI_ID == 4
            return PAMI_4_WAIT;
        #elif PAMI_ID == 5
            return PAMI_5_WAIT;
        #endif
        return 0.0;
    };

    bool position_in_end_zone(RobotPosition position)
    {
        #if PAMI_ID == 1
            return robot_position_in_zone(position, COORD_ZONE_3);
        #elif PAMI_ID == 2
            return robot_position_in_zone(position, COORD_ZONE_3);
        #elif PAMI_ID == 3
            return robot_position_in_zone(position, COORD_ZONE_2);
        #elif PAMI_ID == 4
            return robot_position_in_zone(position, COORD_ZONE_1);
        #endif
        return false;
    }

    bool position_in_avoidance_exclusion(RobotPosition position)
    {
        return false;
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
        startPosition = RobotPosition(PAMI_1_START);
    #elif PAMI_ID == 2
       startPosition = RobotPosition(PAMI_2_START);
    #elif PAMI_ID == 3
       startPosition = RobotPosition(PAMI_3_START);
    #elif PAMI_ID == 4
       startPosition = RobotPosition(PAMI_4_START);
    #elif PAMI_ID == 5
       startPosition = RobotPosition(PAMI_5_START);  
    #endif

        motionController->resetPosition(startPosition, true, true, true);
        positions.clear();
        positions.push_back(startPosition);
       
    #if PAMI_ID == 3

        targetPosition = RobotPosition(1940.0, 1450.0, M_PI);
        RobotPosition tmp = targetPosition;

        tmp = startPosition;
        tmp.x += 350;
        positions.push_back(tmp);
        tmp.x = 840;
        tmp.y = 1250.0;
        positions.push_back(tmp);
        tmp.x = 1640.0;
        tmp.y = 1100.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

    #elif PAMI_ID == 2

        targetPosition = RobotPosition(1773.0, 1372.0, M_PI);

        RobotPosition tmp = targetPosition;
        tmp = startPosition;
        tmp.x += 150;
        positions.push_back(tmp);
        tmp.x = 1210.0;
        tmp.y = 1294.0;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());


    #elif PAMI_ID == 4

        targetPosition = RobotPosition(1670.0, 1470.0, M_PI);

        RobotPosition tmp = startPosition;
        tmp.x += 200;
        tmp.y += 1;
        positions.push_back(tmp);
        tmp.x += 100;
        tmp.y -= 150;
        positions.push_back(tmp);
        tmp.x = 1000;
        tmp.y = 1430;
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 150.0);
        res.insert(res.end(), tv.begin(), tv.end());

    #elif PAMI_ID == 1

        targetPosition = RobotPosition(1078.0, 1438.0, -M_PI_2-M_PI_4);
        
        RobotPosition tmp = startPosition;
        tmp.x += 250;
        positions.push_back(tmp);
        tmp = RobotPosition(776.0, 1380.0, 0.0);
        positions.push_back(tmp);
        positions.push_back(targetPosition);
        
        tv = computeTrajectoryRoundedCorner(tc, positions, 200.0);
        res.insert(res.end(), tv.begin(), tv.end());

    #elif PAMI_ID == 5

        // PAMI should go slower
        tc.maxWheelVelocity *= 0.75;
        tc.maxWheelAcceleration *= 0.75;

        RobotPosition tmp = startPosition;
        tmp.x += 1300.0;
        positions.push_back(tmp);
        tmp.y -= 180;
        positions.push_back(tmp);
        tv = computeTrajectoryRoundedCorner(tc, positions, 100.0);
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
        RobotPosition currentPosition(motionController->getCurrentPosition());
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
#if PAMI_ID == 5
        // Go forward
        float distance = 2300;
        // Movement should be very slow
        tc.maxWheelVelocity = SLOW_APPROACH_WHEEL_VELOCITY;
        TrajectoryVector sl = computeTrajectoryStraightLine(tc, currentPosition, distance);
        res.insert(res.end(), sl.begin(), sl.end());
#endif

//         for (auto traj : res)
//         {
//                 traj->setAvoidanceEnabled(false);
//         }
        return res;
    }
}
