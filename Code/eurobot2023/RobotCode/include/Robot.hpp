#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include <MotionController.hpp>
#include <AbstractRobotBase.hpp>
#include <RobotState.hpp>

#include <Preferences.h>
#include <Message.hpp>

#include <SampledTrajectory.h>
using namespace miam::trajectory;

class Robot
{
private:
    Robot();
public:

    static Robot* getInstance();
    static void init();
    static void startLowLevelLoop();

    RobotState get_current_robot_state();
    PamiReportMessage get_pami_report();
    void update_robot_state();
    void notify_new_message(std::shared_ptr<Message >  message);

    bool matchStarted();

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
    bool trajectory_was_read = false;
    
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

    // Robot state
    RobotState currentRobotState_;

    // Message variables
    std::shared_ptr<Message > newMessage_ = nullptr;
    bool newMessageToRead_ = false;
};

#endif