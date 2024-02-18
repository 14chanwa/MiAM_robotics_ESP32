#include <MotionController.hpp>
#include <parameters.hpp>

// #define DEBUG_MOTIONCONTROLLER_CPP

MotionController::MotionController(SemaphoreHandle_t* xMutex_Serial, RobotParameters parameters) : 
                                        currentPosition_(),
                                        newTrajectories_(),
                                        currentTrajectories_(),
                                        wasTrajectoryFollowingSuccessful_(true),
                                        curvilinearAbscissa_(0.0),
                                        xMutex_Serial_(xMutex_Serial),
                                        parameters_(parameters)
{
    kinematics_ = DrivetrainKinematics(parameters_.wheelRadius,
                                       parameters_.wheelSpacing,
                                       parameters_.wheelRadius,
                                       parameters_.wheelSpacing);

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 0;
    initialPosition.y = 0;
    initialPosition.theta = 0;
    setCurrentPosition(initialPosition);

    // Set PIDs.
    PIDLinear_ = miam::PID(LINEAR_KP, LINEAR_KD, LINEAR_KI, 0.2);
    PIDAngular_ = miam::PID(ROTATION_KP, ROTATION_KD, ROTATION_KI, 0.15);

    // Init controller state
    motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;

    // Create mutex
    xMutex = xSemaphoreCreateMutex();  // crete a mutex object
    xMutex_currentPosition = xSemaphoreCreateMutex();
}

void MotionController::init(RobotPosition const& startPosition)
{
    setCurrentPosition(startPosition);
    currentTime_ = 0.0;
}

miam::trajectory::TrajectoryVector MotionController::getCurrentTrajectories()
{
    return currentTrajectories_;
}

float MotionController::getCurvilinearAbscissa()
{
    return curvilinearAbscissa_;
}

void MotionController::resetPosition(miam::RobotPosition const &resetPosition, bool const &resetX, bool const &resetY, bool const &resetTheta)
{
    setCurrentPosition(resetPosition);
}

bool MotionController::setTrajectoryToFollow(TrajectoryVector const &trajectories)
{
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) { // try to acquire the mutex
        newTrajectories_ = trajectories;
        wasTrajectoryFollowingSuccessful_ = true;
        xSemaphoreGive(xMutex);  // release the mutex
    }
    return true;
}

bool MotionController::waitForTrajectoryFinished()
{
    while (!isTrajectoryFinished())
        vTaskDelay(15 / portTICK_PERIOD_MS); // wait 15 ms
    return wasTrajectoryFollowingSuccessful_;
}

bool MotionController::isTrajectoryFinished()
{
    return (currentTrajectories_.size() == 0 && newTrajectories_.size() == 0);
}

bool MotionController::wasTrajectoryFollowingSuccessful()
{
    return wasTrajectoryFollowingSuccessful_;
}

DrivetrainTarget MotionController::computeDrivetrainMotion(DrivetrainMeasurements const &measurements,
                                                           float const &dt,
                                                           bool const &hasMatchStarted)
{
    // Log input
    currentTime_ += dt;
    WheelSpeed wheelIncrementRad(measurements.motorSpeed);
    wheelIncrementRad.left *= dt;
    wheelIncrementRad.right *= dt;

    // Odometry
    kinematics_.integratePosition(wheelIncrementRad, currentPosition_);

    // BaseSpeed baseSpeed = kinematics_.forwardKinematics(measurements.motorSpeed, true);

    DrivetrainTarget target;

    // Compute slowdown
    slowDownCoeff_ = computeObstacleAvoidanceSlowdown(measurements.vlx_range_detection_mm, hasMatchStarted);
    // slowDownCoeff_ = 1.0;
    clampedSlowDownCoeff_ = std::min(slowDownCoeff_, clampedSlowDownCoeff_ + 0.05f);

    changeMotionControllerState();

    target = resolveMotionControllerState(measurements, dt, hasMatchStarted);

    return target;
}


void MotionController::changeMotionControllerState()
{

    // in all cases, if new trajectories, reset controller
    // Load new trajectory, if needed.
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        if (!newTrajectories_.empty())
        {
            // textlog << "[MotionController] New trajectories, reset controller state to WAIT_FOR_TRAJECTORY" << std::endl;
            motionControllerState_ = CONTROLLER_WAIT_FOR_TRAJECTORY;
        }

        MotionControllerState nextMotionControllerState = motionControllerState_;
        
        if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
        {
            if (!newTrajectories_.empty())
            {
                // textlog << "[MotionController] Start reading trajectories " << std::endl;
                // We have new trajectories, erase the current trajectories and follow the new one.
                currentTrajectories_ = newTrajectories_;
                curvilinearAbscissa_ = 0;
                // avoidanceCount_ = 0;
                trajectoryDone_ = false;
                newTrajectories_.clear();
                // textlog << "[MotionController] Recieved " << currentTrajectories_.size() << " new trajectories from strategy" << std::endl;
                // textlog << "[MotionController] Now tracking: " << currentTrajectories_.at(0)->description_ << std::endl;

                nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
            }
        }
        else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
        {

            // transition to STOP
            if (clampedSlowDownCoeff_ < 1.0e-6)
            {
                timeSinceFirstStopped_ = millis();
                nextMotionControllerState = CONTROLLER_STOP;
            }
            else
            {
                // Load first trajectory, look if we are done following it.
                Trajectory *traj = currentTrajectories_.at(0).get();

                if (curvilinearAbscissa_ > traj->getDuration() || trajectoryDone_)
                {
                    currentTrajectories_.erase(currentTrajectories_.begin());
                    // If we still have a trajectory after that, immediately switch to the next trajectory.
                    if (currentTrajectories_.size() > 0)
                    {
                        // Not optimal, could be improved.
                        traj = currentTrajectories_.at(0).get();
                        curvilinearAbscissa_ = 0.0;
                        // textlog << "[MotionController] Now tracking: " << traj->description_ << std::endl;
                    }
                    else
                    {
                        // textlog << "[MotionController] Trajectory tracking performed successfully" << std::endl;
                    }
                }

                // If we are more than a specified time  after the end of the trajectory, stop it anyway.
                // We hope to have servoed the robot is less than that anyway.
                else if (curvilinearAbscissa_ - trajectoryTimeout_ > traj->getDuration())
                {
                    // textlog << "[MotionController] Timeout on trajectory following" << std::endl;
                    currentTrajectories_.erase(currentTrajectories_.begin());
                    curvilinearAbscissa_ = 0.;
                }
            }

            // transition to WAIT_FOR_TRAJECTORY
            if (currentTrajectories_.empty())
            {
                nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
            }
        }
        else if (motionControllerState_ == CONTROLLER_STOP)
        {
            // in seconds
            float durationSinceFirstStopped = (millis() - timeSinceFirstStopped_) / 1000.0;
            // float durationSinceLastAvoidance = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timeSinceLastAvoidance_).count() / 1000.0;

            // transition to TRAJECTORY_TRACKING
            if (clampedSlowDownCoeff_ > 0.0 && durationSinceFirstStopped > 0.1)
            {
                // Robot was stopped and is ready to start again.
                // Replan and retry trajectory.
                currentTrajectories_.at(0)->replanify(curvilinearAbscissa_);
                curvilinearAbscissa_ = 0;
                // textlog << "[MotionController] "  << "Continue trajectory; replan" << std::endl;
                nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
            }
            else if (durationSinceFirstStopped > 1.0) // && durationSinceLastAvoidance > 1.0
            {
                // transition to WAIT_FOR_TRAJECTORY
                // if (avoidanceCount_ > maxAvoidanceAttempts_)
                // {
                    // textlog << "[MotionController] " << "Trajectory failed: attempted " << avoidanceCount_ << " avoidance" << std::endl;
                    // textlog << "[MotionController] " << "Obstacle still present, canceling trajectory" << std::endl;
                    // Failed to perform avoidance.
                    // Raise flag and end trajectory following.
                    wasTrajectoryFollowingSuccessful_ = false;
                    currentTrajectories_.clear();
                    nextMotionControllerState = CONTROLLER_WAIT_FOR_TRAJECTORY;
                // }
                // // transition to AVOIDANCE
                // else 
                // {
                //     textlog << "[MotionController] " << "Scheduling avoidance" << std::endl;

                //     avoidanceComputationMutex_.lock();
                //     avoidanceComputationScheduled_ = true;
                //     avoidanceComputationMutex_.unlock();

                //     timeSinceLastAvoidance_ = std::chrono::steady_clock::now();
                //     std::cout << "slowDownCoeff_ : " << slowDownCoeff_ << std::endl;
                //     std::cout << "clampedSlowDownCoeff_ : " << clampedSlowDownCoeff_ << std::endl;
                //     std::cout << "avoidanceCount_ : " << avoidanceCount_ << std::endl;

                //     nextMotionControllerState = CONTROLLER_WAIT_FOR_AVOIDANCE;
                // }
            }
        }
        // else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
        // {
        //     if (!avoidanceComputationScheduled_)
        //     {
        //         // transition to  TRAJECTORY_TRACKING
        //         if (avoidanceComputationResult_.getDuration() > 0)
        //         {
        //             textlog << "[MotionController] " << "Performing avoidance" << std::endl;

        //             avoidanceComputationMutex_.lock();
        //             currentTrajectories_ = avoidanceComputationResult_;
        //             avoidanceComputationMutex_.unlock();
        //             curvilinearAbscissa_ = 0.0;

        //             nextMotionControllerState = CONTROLLER_TRAJECTORY_TRACKING;
        //         }
        //         // transition to STOP
        //         else
        //         {
        //             textlog << "[MotionController] " << "Avoidance failed" << std::endl;

        //             nextMotionControllerState = CONTROLLER_STOP;
        //         }
        //         avoidanceComputationMutex_.lock();
        //         avoidanceComputationScheduled_ = false;
        //         avoidanceComputationMutex_.unlock();

        //     }
        // }

        xSemaphoreGive(xMutex);  // release the mutex


        // print and change
        if (motionControllerState_ != nextMotionControllerState)
        {
            std::string current;
            if (motionControllerState_ == CONTROLLER_STOP)
            {
                current = "STOP";
            }
            else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
            {
                current = "TRAJECTORY_TRACKING";
            }
            // else if (motionControllerState_ == CONTROLLER_WAIT_FOR_AVOIDANCE)
            // {
            //     current = "WAIT_FOR_AVOIDANCE";
            // }
            else if (motionControllerState_ == CONTROLLER_WAIT_FOR_TRAJECTORY)
            {
                current = "WAIT_FOR_TRAJECTORY";
            }

            std::string next;
            if (nextMotionControllerState == CONTROLLER_STOP)
            {
                next = "STOP";
            }
            else if (nextMotionControllerState == CONTROLLER_TRAJECTORY_TRACKING)
            {
                next = "TRAJECTORY_TRACKING";
            }
            // else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_AVOIDANCE)
            // {
            //     next = "WAIT_FOR_AVOIDANCE";
            // }
            else if (nextMotionControllerState == CONTROLLER_WAIT_FOR_TRAJECTORY)
            {
                next = "WAIT_FOR_TRAJECTORY";
            }

#ifdef DEBUG_MOTIONCONTROLLER_CPP
            // textlog << "[MotionController] State changed: from " << current << " to " << next << std::endl;
            if (xSemaphoreTake(*xMutex_Serial_, portMAX_DELAY))
            {
                Serial.print("[MotionController] State changed: from ");
                Serial.print(current.c_str());
                Serial.print(" to ");
                Serial.println(next.c_str());
                xSemaphoreGive(*xMutex_Serial_);  // release the mutex
            }
#endif
        }

        motionControllerState_ = nextMotionControllerState;
    }
}

DrivetrainTarget MotionController::resolveMotionControllerState(
    DrivetrainMeasurements const &measurements,
    float const &dt,
    bool const &hasMatchStarted
)
{
    DrivetrainTarget target;
    target.motorSpeed[side::RIGHT] = 0.0;
    target.motorSpeed[side::LEFT] = 0.0;

    targetSpeed_.linear = 0.0;
    targetSpeed_.angular = 0.0;

    if (!hasMatchStarted)
    {
        PIDLinear_.resetIntegral(0.0);
        PIDAngular_.resetIntegral(0.0);
    }
    else if (motionControllerState_ == CONTROLLER_TRAJECTORY_TRACKING)
    {
        // Load first trajectory, look if we are done following it.
        Trajectory *traj = currentTrajectories_.at(0).get();
        curvilinearAbscissa_ += clampedSlowDownCoeff_ * dt;

        // Servo robot on current trajectory.
        trajectoryDone_ = computeMotorTarget(traj, curvilinearAbscissa_, dt, clampedSlowDownCoeff_, measurements, target);
        // if (trajectoryDone && currentTrajectories_.size() == 1)
        // {
        //     textlog << "[MotionController] Trajectory tracking performed successfully" << std::endl;
        //     currentTrajectories_.erase(currentTrajectories_.begin());
        // }
    }

    return target;
}

RobotPosition MotionController::getCurrentPosition()
{
    RobotPosition result;
    // if (xSemaphoreTake(xMutex_currentPosition, portMAX_DELAY))
    // {
        result = currentPosition_;
    //     xSemaphoreGive(xMutex_currentPosition);  // release the mutex
    // }
    return result;
}

void MotionController::setCurrentPosition(RobotPosition position)
{
    // if (xSemaphoreTake(xMutex_currentPosition, portMAX_DELAY))
    // {
        currentPosition_ = position;
    //     xSemaphoreGive(xMutex_currentPosition);  // release the mutex
    // }
}

// void MotionController::setLowAvoidanceZone(RobotPosition lowAvoidanceCenter, float lowAvoidanceRadius)
// {
//     lowAvoidanceZone_ = std::make_pair(lowAvoidanceCenter, lowAvoidanceRadius);
//     lowAvoidanceZoneEnabled_ = true;
//     textlog << "[MotionController] Set low avoidance zone around " << lowAvoidanceCenter << " radius " <<  lowAvoidanceRadius << std::endl;
// }

// void MotionController::disableLowAvoidanceZone()
// {
//     lowAvoidanceZoneEnabled_ = false;
// }


// float MotionController::minDistancePositionToObstacle(RobotPosition position, bool includePersistentObstacles)
// {
//     float minDistanceFromObstacle = 10000;

//     for (auto obstacle : getDetectedObstacles(includePersistentObstacles))
//     {

//         float tmpMin = (std::get<0>(obstacle) - position).norm() - std::get<1>(obstacle);
//         float tmpMinEnd = (std::get<0>(obstacle) - position).norm() - std::get<1>(obstacle);

//         // distance to center of obstacle minus size of the obstacle
//         minDistanceFromObstacle = std::min(
//             minDistanceFromObstacle,
//             tmpMin);
//     }

//     return minDistanceFromObstacle;
// }