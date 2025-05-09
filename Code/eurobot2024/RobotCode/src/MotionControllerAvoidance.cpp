#include <MotionController.hpp>
#include <parameters.hpp>
#include <PointTurn.h>

#define AVOIDANCE_SIDE_LEFT 1
#define AVOIDANCE_SIDE_RIGHT -1

// // PAMI 2 & 3 will go near the border of the table
// // so avoiding left is better...
// #if PAMI_ID == 1 || PAMI_ID == 2 || PAMI_ID == 3
//     #define AVOIDANCE_SIDE AVOIDANCE_SIDE_LEFT
// #else
//     #define AVOIDANCE_SIDE AVOIDANCE_SIDE_RIGHT
// #endif

// Avoidance will try to go back this distance before avoiding
#define AVOIDANCE_DISTANCE_BACKWARDS 60.0f
// Avoidance will try to go around the obstacle at a distance this at an angle M_PI_4
#define AVOIDANCE_DISTANCE_SIDE_OBSTACLE 200.0f
// Avoidance will try to catch up with current trajectory after this distance
#define AVOIDANCE_DISTANCE_FORWARD_CATCHUP 450.0f
// Avoidance will limit the catchup velocity to avoid too brutal discontinuity to this factor times max vel
#define AVOIDANCE_MAX_VELOCITY_CATCHUP_FACTOR 0.5

#define AVOIDANCE_OBSTACLE_RADIUS_MM 150.0f
#define AVOIDANCE_MAX_AVOIDANCE_DISTANCE 400.0f


void MotionController::computeAvoidanceTrajectory(DrivetrainMeasurements const& measurements)
{
    // time
    timeSinceLastAvoidance_ = millis();
    bool proximitySwitchTriggered = //(measurements.left_switch_level == 1) || 
        (measurements.right_switch_level == 1);

    Serial.println(">>>>>>>> Replanify");
    // try to replanify
    // first trajectory to follow will be avoidance trajectory 
    TrajectoryConfig tc = getTrajectoryConfig();
    RobotPosition currentPosition(getCurrentPosition());
    // target position after avoidance
    RobotPosition targetPosition(currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).position);
    float targetVelocity = 0.0f;

    // while end of front trajectory is very close, remove
    while (!currentTrajectories_.empty() && 
        (currentTrajectories_.front()->getEndPoint().position - currentPosition).norm() < AVOIDANCE_DISTANCE_FORWARD_CATCHUP
    )
    {
        currentTrajectories_.erase(currentTrajectories_.begin());
        curvilinearAbscissa_ = 0.0f;

        if (!currentTrajectories_.empty())
        {
            targetPosition = currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).position;
            targetVelocity = currentTrajectories_.front()->getCurrentPoint(curvilinearAbscissa_).linearVelocity;
        }
    }

    // if there is still a trajectory, set curvilinearAbscissa so that the point is far enough
    if (!currentTrajectories_.empty())
    {
        std::shared_ptr<Trajectory > frontTrajectory(currentTrajectories_.front());
        float currentCurvilinearAbscissa = 0.0f;
        while (currentCurvilinearAbscissa < frontTrajectory->getDuration() &&
            (frontTrajectory->getCurrentPoint(currentCurvilinearAbscissa).position - currentPosition).norm() < AVOIDANCE_DISTANCE_FORWARD_CATCHUP
            )
        {
            currentCurvilinearAbscissa += 0.2f;
        }
        // replanify the trajectory and set targetPosition to the start of the remaining trajectory
        if (currentCurvilinearAbscissa < frontTrajectory->getDuration())
        {
            frontTrajectory->replanify(currentCurvilinearAbscissa, tc.maxWheelVelocity * AVOIDANCE_MAX_VELOCITY_CATCHUP_FACTOR);
            curvilinearAbscissa_ = 0.0f;
            targetPosition = frontTrajectory->getCurrentPoint(curvilinearAbscissa_).position;
            targetVelocity = frontTrajectory->getCurrentPoint(curvilinearAbscissa_).linearVelocity;
        }
        // handle the case when the trajectory is at the limit
        else
        {
            targetPosition = frontTrajectory->getEndPoint().position;
            targetVelocity = frontTrajectory->getEndPoint().linearVelocity;
            curvilinearAbscissa_ = 0.0f;
            currentTrajectories_.erase(currentTrajectories_.begin());
        }
    }

    // Make the new trajectory to targetPosition, which is either the end of the whole movement
    // or the start of the replanified trajectory
    TrajectoryVector res;

    // First go back 5 cm if obstacle is too close
    TrajectoryVector avoidanceTrajectory;
    RobotPosition avoidancePositionAfterGoingBack = getCurrentPosition();
    RobotPosition avoidancePositionAfterGoingBackAndPointTurn = avoidancePositionAfterGoingBack;
    if (measurements.vlx_range_detection_mm < AVOIDANCE_DISTANCE_BACKWARDS || proximitySwitchTriggered)
    {
        avoidanceTrajectory = computeTrajectoryStraightLine(tc, currentPosition, -AVOIDANCE_DISTANCE_BACKWARDS);
        avoidancePositionAfterGoingBack =  avoidanceTrajectory.getEndPoint().position;
        res.insert(res.end(), avoidanceTrajectory.begin(), avoidanceTrajectory.end());

        // // then do an arc circle around obstacle, towards avoidance end
        // std::shared_ptr<Trajectory > pointTurnTowardsLeft(
        //     std::make_shared<PointTurn >(
        //         tc, 
        //         avoidancePositionAfterGoingBack, 
        //         avoidancePositionAfterGoingBack.theta + AVOIDANCE_SIDE * M_PI_2)
        // );
        // res.push_back(pointTurnTowardsLeft);
        avoidancePositionAfterGoingBackAndPointTurn = res.back()->getEndPoint().position;
    }

    // Decide on avoidance side based on measurements
    int AVOIDANCE_SIDE = measurements.left_vlx > measurements.right_vlx ? AVOIDANCE_SIDE_LEFT : AVOIDANCE_SIDE_RIGHT;

    // Finally, go around
    // assume object radius is AVOIDANCE_OBSTACLE_RADIUS_MM
    RobotPosition avoidancePoint = avoidancePositionAfterGoingBack;
    float dist = std::sqrt(
        std::pow(std::min(std::max(AVOIDANCE_OBSTACLE_RADIUS_MM, measurements.vlx_range_detection_mm), AVOIDANCE_MAX_AVOIDANCE_DISTANCE), 2) + 
        std::pow(AVOIDANCE_OBSTACLE_RADIUS_MM, 2)
    );
    avoidancePoint.x += dist * std::cos(avoidancePoint.theta + AVOIDANCE_SIDE * M_PI_4);
    avoidancePoint.y += dist * std::sin(avoidancePoint.theta + AVOIDANCE_SIDE * M_PI_4);
    
    std::vector<RobotPosition > positions;
    positions.push_back(avoidancePositionAfterGoingBackAndPointTurn);
    positions.push_back(avoidancePoint);
    positions.push_back(targetPosition);
    TrajectoryVector followingAvoidanceTrajectory = computeTrajectoryRoundedCorner(tc, positions, 150.0f, 0.5f, false, targetVelocity);

    res.insert(res.end(), followingAvoidanceTrajectory.begin(), followingAvoidanceTrajectory.end());

    // disable avoidance for the avoidance trajectories
    for (auto traj : res)
    {
        traj->isAvoidanceTrajectory_ = true;
    }

    // insert remaining trajectories
    res.insert(res.end(), currentTrajectories_.begin(), currentTrajectories_.end());

    // clear
    currentTrajectories_.clear();
    currentTrajectories_.insert(currentTrajectories_.end(), res.begin(), res.end());
    curvilinearAbscissa_ = 0.0f;
    slowDownCoeff_ = 1.0f;
    clampedSlowDownCoeff_ = 1.0f;

}