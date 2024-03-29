#include <Robot.hpp>
#include <parameters.hpp>

#include <RobotBaseDC.hpp>
#include <RobotBaseStepper.hpp>

#include <AnalogReadings.hpp>
#include <ServoHandler.hpp>
#include <I2CHandler.hpp>

#include <Strategy.hpp>

Robot* Robot::getInstance() 
{
    static Robot instance;
    return &instance;
}

void performStrategy(void* parameters)
{
    Robot* robot = Robot::getInstance();

    strategy::perform_strategy(
        robot->motionController,
        strategy::get_waiting_time_s(), 
        robot->saved_trajectory_vector
    );
    Serial.println("Strategy ended");
    vTaskDelete( NULL );
}

void performLowLevel(void* parameters)
{
    Robot* robot = Robot::getInstance();

    long timeStartLoop = 0;
    long timeEndLoop = 0;
    for(;;)
    {
        timeEndLoop = micros();
        robot->dt_period_ms = (timeEndLoop - timeStartLoop) / 1000.0;
        timeStartLoop = timeEndLoop;

        // update match time
        if (robot->match_started)
        {
            robot->match_current_time_s += robot->dt_period_ms / 1000;

            // if match started and trajectory was not read, start
            if (
                !robot->trajectory_was_read && 
                robot->match_current_time_s > MATCH_PAMI_START_TIME_S && 
                robot->match_current_time_s < MATCH_DURATION_S && 
                robot->motionController->isTrajectoryFinished()
            )
            {
                if (robot->saved_trajectory_vector.size() > 0)
                {
                robot->trajectory_was_read = true;
                Serial.println("Beginning match");
                // Start the strategy
                xTaskCreate(
                    performStrategy,
                    "performStrategy",
                    10000,
                    NULL,
                    10,
                    NULL
                );
                }
                else
                {
                Serial.println("No trajectory to perform");
                }
            }

            if (robot->match_current_time_s > MATCH_DURATION_S)
            {
                robot->match_started = false;
                robot->trajectory_was_read = false;
                robot->motionController->clearTrajectories();
            }
        }
        
        // Serial.println("Update sensors");
        robot->robotBase->updateSensors();
        // Serial.println("Get measurements");
        robot->measurements = robot->robotBase->getMeasurements();
        robot->measurements.vlx_range_detection_mm = I2CHandler::get_current_vl53l0x();

        // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
        if (robot->motionController->isPlayingRightSide_)
        {
            float temp = robot->measurements.motorSpeed[side::RIGHT];
            robot->measurements.motorSpeed[side::RIGHT] = robot->measurements.motorSpeed[side::LEFT];
            robot->measurements.motorSpeed[side::LEFT] = temp;
        }

        // Serial.println("Compute drivetrain motion");
        // Motion occurs only if match started
        robot->target = robot->motionController->computeDrivetrainMotion(
            robot->measurements, 
            robot->dt_period_ms / 1000.0, 
            robot->match_started || robot->movement_override
        );

        // invert kinematics
        if (robot->motionController->isPlayingRightSide_)
        {
            float leftSpeed = robot->target.motorSpeed[side::LEFT];
            robot->target.motorSpeed[side::LEFT] = robot->target.motorSpeed[side::RIGHT];
            robot->target.motorSpeed[side::RIGHT] = leftSpeed;
        }

        // Serial.println("Set base speed");
        robot->robotBase->setBaseSpeed(robot->target);

        // update motor control
        // Serial.println("Update motor control");
        robot->robotBase->updateControl();

        // update sensors
        AnalogReadings::update();

        // Serial.println("Register time");
        timeEndLoop = micros();
        robot->dt_lowLevel_ms = (timeEndLoop - timeStartLoop) / 1000.0;

        // wait till next tick (integer division rounds toward 0)
        vTaskDelay( std::max(1L, (timeStartLoop + LOW_LEVEL_LOOP_TIME_MS * 1000 - timeEndLoop) / 1000) / portTICK_PERIOD_MS);
    }
}

Robot::Robot()
{
    #ifdef USE_DC_MOTORS
    robotBase = RobotBaseDC::getInstance();
    #else 
    #ifdef USE_STEPPER_MOTORS
    robotBase = RobotBaseStepper::getInstance();
    #endif
    #endif

    // init robot base
    Serial.println("Create robot base");
    robotBase->setup();

    // Init preferences
    preferences.begin("miam-pami", false); 
    // Load existing preferences
    robotID = preferences.getInt("id", -1);
    if (robotID == -1)
    {
        robotID = PAMI_ID;
    }

    // Init motionController
    motionController = new MotionController(&xMutex_Serial, robotBase->getParameters());
    motionController->init(RobotPosition(0.0, 0.0, 0.0));

    length_of_saved_traj_float = preferences.getInt("traj_len_float", -1);
    duration_of_saved_traj = preferences.getFloat("traj_duration", -1);
    // if (length_of_saved_traj_float > 0)
    // {
    //   Serial.print("Reading saved traj of length ");
    //   Serial.print(length_of_saved_traj_float);
    //   Serial.print(", duration ");
    //   Serial.println(duration_of_saved_traj);
    //   saved_trajectory = new float[length_of_saved_traj_float]();
    //   preferences.getBytes("traj_coord", saved_trajectory, length_of_saved_traj_float*4);
    //   Serial.print("First float ");
    //   Serial.println(saved_trajectory[0]);
    //   Serial.print("Last float ");
    //   Serial.println(saved_trajectory[length_of_saved_traj_float-1]);

    //   std::vector<TrajectoryPoint > tp_vec;
    //   TrajectoryPoint tp;
    //   for (int i = 0; i < length_of_saved_traj_float / 5; i++)
    //   {
    //     tp.position.x = saved_trajectory[5*i];
    //     tp.position.y = saved_trajectory[5*i+1];
    //     tp.position.theta = saved_trajectory[5*i+2];
    //     tp.linearVelocity = saved_trajectory[5*i+3];
    //     tp.angularVelocity = saved_trajectory[5*i+4];
    //     tp_vec.push_back(tp);
    //   }

    //   saved_trajectory_vector.clear();
    //   std::shared_ptr<Trajectory > traj(new SampledTrajectory(tc, tp_vec, duration_of_saved_traj));
    //   saved_trajectory_vector.push_back(traj);
    // }
    // else
    // {
        Serial.println("Load default trajectory");
        saved_trajectory_vector = strategy::get_default_trajectory(motionController);
    // }

    xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object
}

void Robot::init()
{
    Robot::getInstance();
    return;
}

void Robot::startLowLevelLoop()
{
    // begin low level loop
    Serial.println("Launch low level loop");
    xTaskCreatePinnedToCore(
        performLowLevel, 
        "performLowLevel",
        10000,
        NULL,
        19, // high priority
        NULL, 
        0 // pin to core 0
    ); 
}