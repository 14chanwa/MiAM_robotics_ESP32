/// \file trajectory/SampledTrajectory.h
/// \brief A trajectory defined by a set of TrajectoryPoints & linear interpolation between points.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_SAMPLED_TRAJECTORY
#define MIAM_TRAJECTORY_SAMPLED_TRAJECTORY

    #include "Trajectory.h"
    #include <vector>

    namespace miam{
        namespace trajectory{
            class SampledTrajectory: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    ///
                    /// \param[in] sampled_trajectory Vector of trajectory waypoints. Suppose they are equally sampled in time.
                    /// \param[in] start_time Time at the first point of the trajectory.
                    /// \param[in] end_time Time at the last point of the trajectory.
                    SampledTrajectory(
                        TrajectoryConfig const& config,
                        std::vector<TrajectoryPoint > sampledTrajectory,
                        float duration
                        );

                    TrajectoryPoint getCurrentPoint(float const& currentTime);

                    void replanify(float const& replanificationTime, float const& maxStartVelocity = 0.0);

                    void removePoints(int n);

                private:
                    std::vector<TrajectoryPoint > sampledTrajectory_; ///< Vector of trajectory waypoints.
            };
        }
    }
#endif
