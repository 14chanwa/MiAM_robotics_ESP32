#ifndef _STRATEGY_HPP
#define _STRATEGY_HPP

#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <MotionController.hpp>

namespace strategy
{
    void go_forward(MotionController* motionController, float distance);

    void turn_around(MotionController* motionController, float angle);

    void go_to_point(MotionController* motionController, RobotPosition targetPoint);

    void make_a_square(MotionController* motionController);

    void go_to_zone_3(MotionController* motionController);

    void perform_strategy(
        MotionController *motionController,
        float waiting_time_before_start_s, 
        TrajectoryVector trajectory_to_objective
    );

    TrajectoryVector get_default_trajectory(MotionController* motionController);
    TrajectoryVector get_alternative_trajectory(MotionController* motionController);
    TrajectoryVector get_final_action_trajectory(MotionController* motionController);

    float get_waiting_time_s();

    bool position_in_end_zone(RobotPosition position);
    bool position_in_avoidance_exclusion(RobotPosition position);
}


#endif