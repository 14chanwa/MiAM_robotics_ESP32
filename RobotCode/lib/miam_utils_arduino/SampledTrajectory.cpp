/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "SampledTrajectory.h"
#include <cmath>


namespace miam{
    namespace trajectory{
        SampledTrajectory::SampledTrajectory(
            TrajectoryConfig const& config,
            std::vector<TrajectoryPoint > sampledTrajectory,
            float duration
            ) : Trajectory(config)
        {
            description_ = "SampledTrajectory";
            duration_ = duration;
            sampledTrajectory_ = sampledTrajectory;
        }

        TrajectoryPoint SampledTrajectory::getCurrentPoint(float const& currentTime)
        {
            int N = sampledTrajectory_.size();

            if (N < 1)
            {
                return TrajectoryPoint();
            }

            if (currentTime >= duration_)
            {
                return sampledTrajectory_.back();
            }
            else if (currentTime <= 0.0)
            {
                return sampledTrajectory_.front();
            }

            int indexLow = std::floor((N-1) * currentTime / duration_);
            int indexHigh = std::ceil((N-1) * currentTime / duration_);

            TrajectoryPoint tpLow = sampledTrajectory_.at(indexLow);
            TrajectoryPoint tpHigh = sampledTrajectory_.at(indexHigh);

            float sampledTimestep = duration_ / (N-1);

            float residue = (currentTime - indexLow * sampledTimestep) / sampledTimestep;

            float ponderationLow = 1.0 - residue;
            float ponderationHigh = residue;

            // Linear interpolation
            TrajectoryPoint output;
            output.position.x = ponderationLow * tpLow.position.x + ponderationHigh * tpHigh.position.x;
            output.position.y = ponderationLow * tpLow.position.y + ponderationHigh * tpHigh.position.y;
            output.position.theta = ponderationLow * tpLow.position.theta + ponderationHigh * tpHigh.position.theta;
            output.linearVelocity = ponderationLow * tpLow.linearVelocity + ponderationHigh * tpHigh.linearVelocity;
            output.angularVelocity = ponderationLow * tpLow.angularVelocity + ponderationHigh * tpHigh.angularVelocity;

            return output;

        }

        void SampledTrajectory::replanify(float const& replanificationTime, float const& maxStartVelocity)
        {
            int N = sampledTrajectory_.size();

            // case trajectory is finished
            if (replanificationTime >= getDuration() || N == 1)
            {
                sampledTrajectory_.clear();
                sampledTrajectory_.push_back(getCurrentPoint(replanificationTime));
                duration_ = 0.0;
                return;
            }

            float sampling_time = getDuration() / (N-1);
            float currentMaxLinearVelocity = 0.0; // maximum theoretical velocity that could be obtained
            float currentCurvilinearAbscissa = replanificationTime; // time in the old traj
            float currentScalingFactor = 0.0; // factor by which to slowdown the traj

            // initial update scale factor
            currentMaxLinearVelocity = std::max(maxStartVelocity, getCurrentPoint(replanificationTime).linearVelocity);
            currentScalingFactor = currentMaxLinearVelocity / config_.maxWheelVelocity;

            TrajectoryPoint tp;
            std::vector<TrajectoryPoint > newSampledTrajectory;
            
            while (currentCurvilinearAbscissa < getDuration())
            {
                // compute new trajectory point, scaling velocities
                tp = getCurrentPoint(currentCurvilinearAbscissa);
                tp.linearVelocity *= currentScalingFactor;
                tp.angularVelocity *= currentScalingFactor;
                newSampledTrajectory.push_back(tp);

                // prepare next point
                currentMaxLinearVelocity = std::min(
                    config_.maxWheelVelocity,
                    currentMaxLinearVelocity + config_.maxWheelAcceleration * sampling_time
                );
                currentScalingFactor = currentMaxLinearVelocity / config_.maxWheelVelocity;
                currentCurvilinearAbscissa += sampling_time * std::pow(currentScalingFactor, 2);
            }

            // finally, add last point to end trajectory properly
            newSampledTrajectory.push_back(getEndPoint());

            duration_ = (newSampledTrajectory.size() - 1) * sampling_time;
            sampledTrajectory_ = newSampledTrajectory;
        }

        void SampledTrajectory::removePoints(int n)
        {
            if (n <= 0)
            {
                return;
            }

            if (sampledTrajectory_.size() - n <= 0)
            {
                sampledTrajectory_.clear();
                duration_ = 0.0;
            }

            float dt = duration_ / (sampledTrajectory_.size() - 1);
            for (int i = 0; i < n; i++)
            {
                sampledTrajectory_.pop_back();
            }
            duration_ = std::max(0.0f, duration_ - n * dt);
        }
    }
}
