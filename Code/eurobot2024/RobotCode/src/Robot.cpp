#include <Robot.hpp>
#include <parameters.hpp>

#include <RobotBaseDC.hpp>
#include <RobotBaseStepper.hpp>

#include <AnalogReadings.hpp>
#include <ServoHandler.hpp>
#include <I2CHandler.hpp>

#include <Strategy.hpp>

using namespace miam::trajectory;

#define FUNNY_ACTION_SERVO_PERIOD 500
bool funny_action_state = false;
long funny_action_timer = 0;


Robot* Robot::getInstance() 
{
    static Robot instance;
    return &instance;
}

// void performStrategy(void* parameters)
// {
//     Robot* robot = Robot::getInstance();

//     strategy::perform_strategy(
//         robot->motionController,
//         strategy::get_waiting_time_s(), 
//         robot->saved_trajectory_vector
//     );
//     Serial.println("Strategy ended");
//     vTaskDelete( NULL );
// }

void performLowLevel(void* parameters)
{
    Robot* robot = Robot::getInstance();

    long timeStartLoop = 0;
    long timeEndLoop = 0;
    for(;;)
    {
        timeEndLoop = micros();
        // Time since function was last called
        robot->dt_period_ms = (timeEndLoop - timeStartLoop) / 1000.0;
        timeStartLoop = timeEndLoop;

        // update match time
        if (robot->matchStarted() && robot->match_current_time_s < 100.0)
        {
            robot->match_current_time_s += robot->dt_period_ms / 1000;
        }

        // update match state
        robot->update_robot_state();
        
        // Serial.println("Update sensors");
        robot->robotBase->updateSensors();
        // Serial.println("Get measurements");
        robot->measurements = robot->robotBase->getMeasurements();
        robot->measurements.vlx_range_detection_mm = I2CHandler::get_smoothed_vl53l0x();
        robot->measurements.left_vlx = I2CHandler::get_smoothed_vlx_side(I2CHandler::Side::LEFT);
        robot->measurements.middle_vlx = I2CHandler::get_smoothed_vlx_side(I2CHandler::Side::MIDDLE);
        robot->measurements.right_vlx = I2CHandler::get_smoothed_vlx_side(I2CHandler::Side::RIGHT);
        robot->measurements.left_switch_level = AnalogReadings::get_left_switch_value();
        robot->measurements.right_switch_level = AnalogReadings::get_right_switch_value();
        robot->measurements.currentRobotState = robot->get_current_robot_state();
        robot->measurements.currentMatchTime = robot->matchStarted() ? robot->match_current_time_s : 0.0f;

        // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
        if (robot->motionController->isPlayingRightSide_)
        {
            float temp = robot->measurements.motorSpeed[side::RIGHT];
            robot->measurements.motorSpeed[side::RIGHT] = robot->measurements.motorSpeed[side::LEFT];
            robot->measurements.motorSpeed[side::LEFT] = temp;
        }

        bool robotEnabled = (robot->currentRobotState_ == RobotState::MATCH_STARTED_ACTION ||
                robot->currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH ||
                robot->currentRobotState_ == RobotState::MOVING_SETUP_TRAJECTORY); // &&
                //(!robot->measurements.left_switch_level && !robot->measurements.right_switch_level);

        // Serial.println("Compute drivetrain motion");
        // Motion occurs only if match started
        robot->target = robot->motionController->computeDrivetrainMotion(
            robot->measurements, 
            robot->dt_period_ms / 1000.0, 
            // Robot will stop if not enabled
            robotEnabled,
            // Avoidance is disabled if final approach
            (robot->currentRobotState_ != RobotState::MATCH_STARTED_FINAL_APPROACH) && (robot->currentRobotState_ != RobotState::MOVING_SETUP_TRAJECTORY) 
        );

        // Serial.println("Set base speed");
        robot->robotBase->setBaseSpeed(robot->target);

        // update motor control
        // Serial.println("Update motor control");
        robot->robotBase->updateControl(robotEnabled);

        // handle servo: servo is down iff
        // * MATCH_STARTED_FINAL_APPROACH and currentTime >= 99s
        // * MATCH_ENDED
        if (
            (robot->currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH && robot->match_current_time_s >= 99.0f)
            || robot->currentRobotState_ == RobotState::MATCH_ENDED 
        )
        {
            if (millis() - funny_action_timer > FUNNY_ACTION_SERVO_PERIOD)
            {
                funny_action_state = !funny_action_state;
                funny_action_timer = millis();
            }
            if (funny_action_state)
            {
                ServoHandler::servoDown();
            }
            else
            {
                ServoHandler::servoUp();
            }
        }
        else
        {
            ServoHandler::servoUp();
        }

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

    length_of_saved_traj_float = -1; //preferences.getInt("traj_len_float", -1);
    duration_of_saved_traj = -1; //preferences.getFloat("traj_duration", -1);
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
        // // Transform into SampledTrajectory
        // float duration = saved_trajectory_vector.getDuration();
        // std::vector<TrajectoryPoint> points;
        // float currentTime = 0.0;
        // points.push_back(saved_trajectory_vector.getCurrentPoint(currentTime));
        // while (currentTime < duration)
        // {
        //     currentTime = std::min(currentTime + 0.1f, duration);
        //     points.push_back(saved_trajectory_vector.getCurrentPoint(currentTime));
        // }

        // TrajectoryConfig tc = motionController->getTrajectoryConfig();
        // match_trajectory = std::make_shared<SampledTrajectory >(tc, points, duration);
    // }

    xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object

    currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
}

SemaphoreHandle_t xMutex_newMessage = NULL;
bool robotWasInit = false;

void Robot::init()
{
    if (!robotWasInit)
    {
        xMutex_newMessage = xSemaphoreCreateMutex();
    }
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

RobotState Robot::get_current_robot_state()
{
    return currentRobotState_;
}

void Robot::update_robot_state()
{
    // handle message
    // read the message (even if it is not going to be handled afterwards)
    std::shared_ptr<Message > message = nullptr;
    bool newMessageRead = false;
    if (xSemaphoreTake(xMutex_newMessage, portMAX_DELAY))
    {
        if (newMessageToRead_)
        {
            message = newMessage_;
            newMessageToRead_ = false;
            newMessageRead = true;
        }
        xSemaphoreGive(xMutex_newMessage);
    }

    // handle transitions
    // WAIT_FOR_CONFIGURATION
    if (currentRobotState_ == RobotState::WAIT_FOR_CONFIGURATION)
    {
        // WAIT_FOR_MATCH_START
        if (newMessageRead && message->get_message_type() == MessageType::CONFIGURATION)
        {
            ConfigurationMessage* configurationMessage = static_cast<ConfigurationMessage* >(message.get());
            Serial.println(">> WAIT_FOR_CONFIGURATION -> WAIT_FOR_MATCH_START");
            // Blue side is left side
            // So right side is yellow
            motionController->isPlayingRightSide_ = configurationMessage->playingSide_ == PlayingSide::YELLOW_SIDE;
            // Stop motors?
#ifdef USE_STEPPER_MOTORS
            static_cast<RobotBaseStepper*>(robotBase)->setBlockWheels(configurationMessage->stopMotors_);
#endif
            currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
        }
    }

    // WAIT_FOR_MATCH_START
    else if (currentRobotState_ == RobotState::WAIT_FOR_MATCH_START)
    {
        // MOVING_TRAJECTORY_SETUP
        if (newMessageRead && message->get_message_type() == MessageType::NEW_TRAJECTORY)
        {
            NewTrajectoryMessage* newTrajectoryMessage = static_cast<NewTrajectoryMessage* >(message.get());
            Serial.println(">> WAIT_FOR_MATCH_START -> MOVING_TRAJECTORY_SETUP");
            // Reset position
            motionController->resetPosition(newTrajectoryMessage->newTrajectory_.getCurrentPoint(0).position, true, true, true);
            // Set new trajectory new trajectory
            motionController->setTrajectoryToFollow(newTrajectoryMessage->newTrajectory_);
            // Update state
            currentRobotState_ = RobotState::MOVING_SETUP_TRAJECTORY;
        }
        
        // WAIT_FOR_MATCH_START
        else if (newMessageRead && message->get_message_type() == MessageType::CONFIGURATION)
        {
            ConfigurationMessage* configurationMessage = static_cast<ConfigurationMessage* >(message.get());
            Serial.println(">> WAIT_FOR_MATCH_START -> WAIT_FOR_MATCH_START");
            // Blue side is left side
            // So right side is yellow
            motionController->isPlayingRightSide_ = configurationMessage->playingSide_ == PlayingSide::YELLOW_SIDE;
            // Stop motors?
#ifdef USE_STEPPER_MOTORS
            static_cast<RobotBaseStepper*>(robotBase)->setBlockWheels(configurationMessage->stopMotors_);
#endif
            currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
        }

        // MATCH_STARTED_WAITING
        else if (newMessageRead && message->get_message_type() == MessageType::MATCH_STATE)
        {
            MatchStateMessage* matchStateMessage = static_cast<MatchStateMessage* >(message.get());
            // Only transition if match started
            if (matchStateMessage->matchStarted_)
            {
                Serial.println(">> WAIT_FOR_MATCH_START -> MATCH_STARTED_WAITING");
                match_current_time_s = matchStateMessage->matchTime_;
                currentRobotState_ = RobotState::MATCH_STARTED_WAITING;
            }
        }
    }

    // MOVING_SETUP_TRAJECTORY
    else if (currentRobotState_ == RobotState::MOVING_SETUP_TRAJECTORY)
    {
        // WAIT_FOR_MATCH_START
        if (motionController->isTrajectoryFinished())
        {
            Serial.println(">> MOVING_SETUP_TRAJECTORY -> WAIT_FOR_MATCH_START");
            currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
        }
    }

    // MATCH_STARTED_WAITING
    else if (currentRobotState_ == RobotState::MATCH_STARTED_WAITING)
    {

        // MATCH_STARTED_WAITING
        // and
        // WAIT_FOR_MATCH_START
        if (newMessageRead && message->get_message_type() == MessageType::MATCH_STATE)
        {
            MatchStateMessage* matchStateMessage = static_cast<MatchStateMessage* >(message.get());
            // If match started, update time
            if (matchStateMessage->matchStarted_)
            {
                Serial.println(">> MATCH_STARTED_WAITING -> MATCH_STARTED_WAITING");
                match_current_time_s = matchStateMessage->matchTime_;
                currentRobotState_ = RobotState::MATCH_STARTED_WAITING;
            }
            // If match not started, update state
            else
            {
                Serial.println(">> MATCH_STARTED_WAITING -> WAIT_FOR_MATCH_START");
                currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
            }
        }

        // MATCH_STARTED_ACTION
        else if (match_current_time_s >= MATCH_PAMI_START_TIME_S + strategy::get_waiting_time_s())
        {
            Serial.println(">> MATCH_STARTED_WAITING -> MATCH_STARTED_ACTION");
#ifdef USE_STEPPER_MOTORS
            static_cast<RobotBaseStepper*>(robotBase)->setBlockWheels(true);
#endif

            TrajectoryVector tv;

            // Query alternative if road is blocked
            // get measurement
            if (I2CHandler::get_smoothed_vl53l0x() <= 600.0f)
            {
                tv.clear();
                tv = strategy::get_alternative_trajectory(motionController);
            }
            else
            {
                // Sets travel to objective
                motionController->resetPosition(saved_trajectory_vector.getCurrentPoint(0.0f).position, true, true, true);
                tv = saved_trajectory_vector;
            }

            motionController->setTrajectoryToFollow(tv);
            currentRobotState_ = RobotState::MATCH_STARTED_ACTION;
        }
    }

    // MATCH_STARTED_ACTION
    else if (currentRobotState_ == RobotState::MATCH_STARTED_ACTION)
    {
        // MATCH_ENDED
        if (match_current_time_s >= 100.0)
        {
            Serial.println(">> MATCH_STARTED_ACTION -> MATCH_ENDED");
            motionController->clearTrajectories();
#ifdef USE_STEPPER_MOTORS
            static_cast<RobotBaseStepper*>(robotBase)->setBlockWheels(true);
#endif
            currentRobotState_ = RobotState::MATCH_ENDED;
        }
        
        // MATCH_STARTED_FINAL_APPROACH
        // or
        // MATCH_STARTED_ACTION
        else if (motionController->isTrajectoryFinished())
        {
            // if (motionController->wasTrajectoryFollowingSuccessful())
            // {
                Serial.println(">> MATCH_STARTED_ACTION -> MATCH_STARTED_FINAL_APPROACH");
                TrajectoryVector tv = strategy::get_final_action_trajectory(motionController);
                motionController->setTrajectoryToFollow(tv);
                currentRobotState_ = RobotState::MATCH_STARTED_FINAL_APPROACH;
            // }
            // else
            // {
            //     Serial.println(">> MATCH_STARTED_ACTION -> MATCH_STARTED_ACTION");
            //     // Attempt straight line + point turn
            //     TrajectoryConfig tc = motionController->getTrajectoryConfig();
            //     RobotPosition curPos(motionController->getCurrentPosition());
            //     // straight line
            //     TrajectoryVector tv(computeTrajectoryStraightLineToPoint(tc, curPos, saved_trajectory_vector.getEndPoint().position));
            //     // point turn
            //     std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, saved_trajectory_vector.getEndPoint().position.theta));
            //     tv.push_back(pt);
            //     for (auto traj : tv)
            //     {
            //         traj->setAvoidanceEnabled(false);
            //     }
            //     motionController->setTrajectoryToFollow(tv);
            // }
        }
    }

    // MATCH_STARTED_FINAL_APPROACH
    else if (currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH)
    {
        // MATCH_ENDED
        if (match_current_time_s >= 100.0)
        {
            Serial.println(">> MATCH_STARTED_FINAL_APPROACH -> MATCH_ENDED");
            motionController->clearTrajectories();
            currentRobotState_ = RobotState::MATCH_ENDED;
        }
    }

    // MATCH_ENDED
    else if (currentRobotState_ == RobotState::MATCH_ENDED)
    {
        if (newMessageRead && message->get_message_type() == MessageType::MATCH_STATE)
        {
            MatchStateMessage* matchStateMessage = static_cast<MatchStateMessage* >(message.get());
            // If match not started, update state
            if (!matchStateMessage->matchStarted_)
            {
                Serial.println(">> MATCH_ENDED -> WAIT_FOR_MATCH_START");
#ifdef USE_STEPPER_MOTORS
            static_cast<RobotBaseStepper*>(robotBase)->setBlockWheels(false);
#endif
                currentRobotState_ = RobotState::WAIT_FOR_MATCH_START;
            }
        }
    }
}

void Robot::notify_new_message(std::shared_ptr<Message > message)
{
    if (xSemaphoreTake(xMutex_newMessage, portMAX_DELAY))
    {
        Serial.println("Notifying new message");
        newMessage_ = message;
        newMessageToRead_ = true;
        lastMessageReceivedTime_ = millis();
        xSemaphoreGive(xMutex_newMessage);
    }
}


bool Robot::matchStarted()
{
    return currentRobotState_ == RobotState::MATCH_STARTED_WAITING ||
        currentRobotState_ == RobotState::MATCH_STARTED_ACTION ||
        currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH ||
        currentRobotState_ == RobotState::MATCH_ENDED;
}

PamiReportMessage Robot::get_pami_report()
{
    RobotState state = get_current_robot_state();
    bool matchStarted = state == RobotState::MATCH_STARTED_WAITING ||
        state == RobotState::MATCH_STARTED_ACTION ||
        state == RobotState::MATCH_STARTED_FINAL_APPROACH ||
        state == RobotState::MATCH_ENDED;
    return(
        PamiReportMessage(
            matchStarted, 
            match_current_time_s, 
            motionController->isPlayingRightSide_ ? PlayingSide::YELLOW_SIDE : PlayingSide::BLUE_SIDE, 
            AnalogReadings::get_current_battery_reading(),
            PAMI_ID
        )
    );
}