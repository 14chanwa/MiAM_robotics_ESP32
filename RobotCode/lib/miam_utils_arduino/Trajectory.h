/// \file trajectory/Trajectory.h
/// \brief Definition of a trajectory, and associated elements.
///
/// \details What we call a trajectory is an object along wich we can servo the robot. It thus groups together the
///          notion of path (i.e. curve on the table, including robot angle), and of time parametrization (in order
///          to allow trajectory tracking in itself).
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_TRAJECTORY
#define MIAM_TRAJECTORY_TRAJECTORY

    #include "RobotPosition.h"
    #include <string>

    namespace miam{
        namespace trajectory{

            // Config variables, used for trajectory generation.
            struct TrajectoryConfig{

                float maxWheelVelocity = 300.0;    // Wheel max velocity, in mm/s
                float maxWheelAcceleration = 300.0;  // Wheel max velocity, in mm/s2
                float robotWheelSpacing = 100.0;  // Wheel spacing, in mm
            };

            /// \brief A trajectory point, containing everything for servoing along this trajectory.
            struct TrajectoryPoint{
                RobotPosition position; ///< Trajectory point in the table.
                float linearVelocity; ///< Linear velocity along the trajectory, at the current point.
                float angularVelocity; ///< Angular velocity along the trajectory, at the current point.

                /// \brief Default constructor.
                TrajectoryPoint();
            };

            class Trajectory
            {
                public:
                    Trajectory(TrajectoryConfig const& config);

                    /// \brief Get trajectory point at current time.
                    ///
                    /// \param[in] currentTime Time relative to trajectory start, in seconds.
                    /// \return The current trajectory point.
                    virtual TrajectoryPoint getCurrentPoint(float const& currentTime) = 0;

                    /// \brief Get trajectory duration, in seconds.
                    /// \return Trajectory duration.
                    float getDuration();

                    /// \brief Get final point of the trajectory.
                    /// \return Position at duration_
                    TrajectoryPoint getEndPoint();

                    /// \brief Replanify the trajectory from the given time.
                    ///
                    /// \details Given a time t, replanify the trajectory to have a new trajectory
                    ///          starting at getCurrentPoint(t) with zero velocity at time t=0.
                    /// \param[in] replanificationTime Time at which we replanify, in seconds.
                    virtual void replanify(float const& replanificationTime, bool resetVelocity = true) = 0;

                    /// \brief Set trajectory avoidance flag
                    void setAvoidanceEnabled(bool avoidanceEnabled) {
                        avoidanceEnabled_ = avoidanceEnabled;
                    };

                    bool isAvoidanceEnabled() {
                        return(avoidanceEnabled_);
                    };

                    std::string description_ = "AbstractTrajectory";
                    bool isAvoidanceTrajectory_;
                    bool needReplanning_;
                protected:
                    float duration_; ///< Trajectory duration.
                    TrajectoryConfig config_;
                    bool avoidanceEnabled_;
            };
        }
    }
#endif
