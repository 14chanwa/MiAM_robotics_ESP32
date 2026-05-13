#ifndef _STRATEGY_HPP
#define _STRATEGY_HPP

#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <MotionController.hpp>

class StrategyPlanner
{
public:
    StrategyPlanner();

    void clear() {
        trajectory_.clear();
    };

    void go_forward(float distance);
    void turn_around(float angle);
    void turn_to_angle(float angle);
    void go_to_point(RobotPosition targetPoint);

    void execute();

    TrajectoryVector get_trajectory() {
        return trajectory_;
    };

    void add_to_trajectory(TrajectoryVector tv);
    RobotPosition get_last_position();

    MotionController* motionController;
    TrajectoryVector trajectory_;
    TrajectoryConfig tc;
};

namespace strategy
{
    bool getHasObjectInSuction();
    void loadObjectInArm();
    void freeObjectFromArm();

    void go_forward(MotionController* motionController, float distance);

    void turn_around(MotionController* motionController, float angle);
    void turn_to_angle(MotionController* motionController, float angle);

    void go_to_point(MotionController* motionController, RobotPosition targetPoint);

    void make_a_square(MotionController* motionController);

    void go_to_zone_3(MotionController* motionController);

    void perform_strategy();

    TrajectoryVector get_default_trajectory(MotionController* motionController);
    TrajectoryVector get_alternative_trajectory(MotionController* motionController);
    TrajectoryVector get_final_action_trajectory(MotionController* motionController);

    float get_waiting_time_s();

    bool position_in_end_zone(RobotPosition position);
    bool position_in_avoidance_exclusion(RobotPosition position);
}


#endif