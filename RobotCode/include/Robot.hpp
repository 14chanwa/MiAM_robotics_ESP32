#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include <MotionController.hpp>
#include <AbstractRobotBase.hpp>

#include <Preferences.h>

class Robot
{
private:
    Robot();
public:

    static Robot* getInstance();
    static void init();
    static void startLowLevelLoop();
    
    ///////////////////////////////////////////////////
    // Variables
    ///////////////////////////////////////////////////

    // RobotBase
    AbstractRobotBase* robotBase;

    // Preferences
    Preferences preferences;

    // Robot ID
    int robotID = 0;
    int length_of_saved_traj_float = 0;
    float duration_of_saved_traj = 0.0f;
    float* saved_trajectory;

    bool match_started = false;
    bool trajectory_was_read = false;
    bool movement_override = false;
    float match_current_time_s = 0.0f;

    TrajectoryVector saved_trajectory_vector;

    // Motion controller
    MotionController* motionController;

    // low level loop timing
    float dt_lowLevel_ms = 0.0;
    float dt_period_ms = 0.0;

    DrivetrainMeasurements measurements;
    DrivetrainTarget target;

    // Semaphores
    SemaphoreHandle_t xMutex_Serial = NULL;

};

#endif