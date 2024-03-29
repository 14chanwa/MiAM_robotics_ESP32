#ifndef _STRATEGY_HPP
#define _STRATEGY_HPP

#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <MotionController.hpp>

namespace strategy
{
    enum MatchState
    {
        WAIT_FOR_CONFIGURATION,
        WAIT_FOR_MATCH_START,
        MATCH_STARTED_WAITING,
        MATCH_STARTED_ACTION,
        MATCH_STARTED_FINAL_APPROACH,
        MATCH_ENDED
    };

    MatchState get_current_match_state();
    void set_current_match_state(MatchState matchState);

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

    float get_waiting_time_s();
}


#endif