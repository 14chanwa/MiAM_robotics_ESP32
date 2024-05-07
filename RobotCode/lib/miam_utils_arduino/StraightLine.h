/// \file trajectory/StraightLine.h
/// \brief A trajectory going in a straight line from two points, along a velocity trapezoid.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_STRAIGHT_LINE
#define MIAM_TRAJECTORY_STRAIGHT_LINE

    #include "Trajectory.h"
    #include "Trapezoid.h"

    namespace miam{
        namespace trajectory{
            class StraightLine: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    /// \details Robot angle along straight line is the angle of the line, computed to be the closest
                    ///          (modulto 2 pi) to the startPoint angle.
                    ///
                    /// \param[in] startPoint Strating point.
                    /// \param[in] endPoint Ending point, only x and y are taken into account.
                    /// \param[in] startVelocity Start velocity.
                    /// \param[in] endVelocity Desired end velocity.
                    /// \param[in] backward If robot should move backward along the straight line.
                    /// \param[in] maxVelocity Max velocity. Only absolute value is taken into account.
                    /// \param[in] maxAcceleration Max acceleration. Only absolute value is taken into account.
                    StraightLine(TrajectoryConfig const& config,
                                 RobotPosition const& startPoint,
                                 RobotPosition const& endPoint,
                                 float const& startVelocity = 0.0,
                                 float const& endVelocity = 0.0,
                                 bool const& backward = false);

                    TrajectoryPoint getCurrentPoint(float const& currentTime);

                    void replanify(float const& replanificationTime, bool resetVelocity = true);

                    /// \brief Return line angle.
                    ///
                    /// \return Line angle
                    float getAngle();
                private:
                    void make(RobotPosition const& startPoint, float const& startVelocity); ///< Build (or rebuild) the trajectory.

                    RobotPosition startPoint_; ///< Point where the trajectory started.
                    int motionSign_; ///< 1 or -1, to indicate direction of motion.
                    Trapezoid trapezoid_; ///< Velocity trapezoid.

                    RobotPosition endPoint_; ///< End position.
                    float endVelocity_; ///< End velocity.
                    bool backward_;     ///< True if going backward.
            };
        }
    }
#endif
