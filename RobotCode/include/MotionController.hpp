/// \file MotionController.h
/// \brief Drivetrain control: odometry, obstacle avoidance, motion control
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <DrivetrainKinematics.h>
#include <RobotPosition.h>
#include <Trajectory.h>
#include <Utilities.h>
#include <PID.h>
#include "RobotParameters.hpp"
#include "AbstractRobotBase.hpp"

using namespace miam;
using namespace miam::trajectory;


enum MotionControllerState {
    CONTROLLER_STOP                 = 0,
    CONTROLLER_TRAJECTORY_TRACKING  = 1,
    // CONTROLLER_WAIT_FOR_AVOIDANCE   = 2,
    CONTROLLER_WAIT_FOR_TRAJECTORY  = 3
};


class MotionController
{

    public:
        MotionController(SemaphoreHandle_t* xMutex_Serial, RobotParameters parameters);

        /// \brief Initialize the system - this also starts the logger.
        /// \param[in] RobotPosition Starting position
        /// \param[in] teleplotPrefix Optional prefix for variables in teleplot: used in simulation
        ///                           where several robots are logging.
        void init(RobotPosition const& startPosition);

        /// \brief Reset the position of the robot on the table.
        ///
        /// \details This function might be used for example when the robot is put in contact with a side of the table,
        ///             to gain back absolute position accuracy.
        ///
        /// \param[in] resetPosition The position to which reset the robot.
        /// \param[in] resetX Wheather or not to reset the X coordinate.
        /// \param[in] resetY Wheather or not to reset the Y coordinate.
        /// \param[in] resetTheta Wheather or not to reset the angle.
        void resetPosition(RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta);

        /// \brief Set new trajectory set to follow.
        /// \details This function is used to set the trajectories which will be followed by
        ///          the low-level thread. This function simply gives the input to the low-level thread
        ///          and returns immediately: use waitForTrajectoryFinish
        ///
        /// \param[in] trajectories Vector of trajectory to follow.
        bool setTrajectoryToFollow(TrajectoryVector const& trajectories);

        /// \brief Wait for the current trajectory following to be finished.
        /// \return true if trajectory following was successful, false otherwise.
        bool waitForTrajectoryFinished();


        /// \brief Get status of last trajectory following.
        bool isTrajectoryFinished();

        /// \brief Get status of last trajectory following.
        bool wasTrajectoryFollowingSuccessful();

        void clearTrajectories();

        /// Enables or disables slow approach :
        /// * speed is very slow
        /// * vlx is not taken into account
        /// * move is stopped when contact is made
        // void setSlowApproach(bool enabled);
        // bool isSlowApproach();
        TrajectoryConfig getTrajectoryConfig();


        /// \brief Compute next motor target.
        ///
        /// \param[in] measurements Latest robot measurements
        /// \param[in] dt Elapsed time since last call.
        /// \return Target motor velocity
        DrivetrainTarget computeDrivetrainMotion(DrivetrainMeasurements const& measurements,
                                                    float const& dt,
                                                    bool const& hasMatchStarted,
                                                    bool const& enableAvoidance);

        void computeAvoidanceTrajectory(DrivetrainMeasurements const& measurements);

        bool isPlayingRightSide_ = false;

        DrivetrainKinematics getKinematics()
        {
            return kinematics_;
        }

        RobotParameters getParameters()
        {
            return parameters_;
        }

        TrajectoryVector getCurrentTrajectories();

        RobotPosition getCurrentPosition();
        void setCurrentPosition(RobotPosition position);

        float getCurvilinearAbscissa();

        BaseSpeed targetSpeed_;
        TrajectoryPoint targetPoint;

        float slowDownCoeff_ = 1.0;
        float clampedSlowDownCoeff_ = 1.0;
    private:
        float currentTime_{0.0};
        RobotPosition currentPosition_; ///< Current robot position,

        // Trajectory definition.
        TrajectoryVector newTrajectories_; ///< Vector of new trajectories to follow.
        TrajectoryVector currentTrajectories_; ///< Current trajectories being followed.
        bool wasTrajectoryFollowingSuccessful_; ///< Flag describing the success of the trajectory following process.

        SemaphoreHandle_t* xMutex_Serial_;  // Create a mutex object
        SemaphoreHandle_t xMutex = NULL;  // Create a mutex object
        SemaphoreHandle_t xMutex_currentPosition = NULL;

        float curvilinearAbscissa_; ///< Curvilinear abscissa of the current trajectory.
        DrivetrainKinematics kinematics_;
        RobotParameters parameters_;

        // Tracking PIDs
        miam::PID PIDLinear_; ///< Longitudinal PID.
        miam::PID PIDAngular_; ///< Angular PID.

        /// \brief Follow a trajectory.
        /// \details This function computes motor velocity to reach a specific trajectory point, and sends
        ///          it to the motors.
        /// \param[in] traj Current trajectory to follow.
        /// \param[in] timeInTrajectory Current time since the start of the trajectory.
        /// \param[in] dt Time since last servoing call, for PID controller.
        /// \param[out] target Motor target
        /// \return True if trajectory following should continue, false if trajectory following is completed.
        bool computeMotorTarget(Trajectory *traj,
                                float const& timeInTrajectory,
                                float const& dt,
                                float const& slowDownRatio,
                                DrivetrainMeasurements const &measurements,
                                DrivetrainTarget &target);

        // /// \brief Updates the LiDAR and sets the avoidance strategy
        // /// \param [in] detectedRobots Obstacles detected by the lidar.
        // /// \return coefficient for trajectory time increase
        float computeObstacleAvoidanceSlowdown(float vlx_range_detection_mm, bool const& hasMatchStarted);

        // RobotPosition lidarPointToRobotPosition(LidarPoint const &point);
        // bool isLidarPointWithinTable(LidarPoint const& point);

        bool isStopped_;
        bool trajectoryDone_ = false;

        long timeSinceFirstStopped_;
        long timeSinceLastAvoidance_ = 0;

        float trajectoryTimeout_ = 1.0; // Number of seconds after the end of trajectory after which timeout is raised

        // Motion controller state
        MotionControllerState motionControllerState_;
        void changeMotionControllerState();
        DrivetrainTarget resolveMotionControllerState(DrivetrainMeasurements const &measurements,
                                                        float const &dt,
                                                        bool const &hasMatchStarted);
      

        // bool slowApproach_ = false;
};
#endif