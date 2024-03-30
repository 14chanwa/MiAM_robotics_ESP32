#include <Message.hpp>
#include <SampledTrajectory.h>

#define MESSAGE_PAYLOAD_START 1

using namespace miam::trajectory;

std::shared_ptr<Message > Message::parse(std::vector<float > message, uint8_t senderId)
{
    // Message type is the first float casted to int
    int message_type = (int)message.at(0);

    // // Sender is the second float casted to int
    // int sender_id = (int) message.at(1);

    if (message_type == MessageType::CONFIGURATION)
    {
        // Check size of payload
        if (message.size() - MESSAGE_PAYLOAD_START == 1)
        {
            // First byte of payload is configuration
            return std::make_shared<ConfigurationMessage >(
                (bool) message.at(MESSAGE_PAYLOAD_START)
            );
        }
    }
    else if (message_type == MessageType::MATCH_STATE)
    {
        // Check size of payload
        if (message.size() - MESSAGE_PAYLOAD_START == 2)
        {
            // First byte of payload is match started
            // Second byte of payload is match time
            return std::make_shared<MatchStateMessage >(
                (bool) message.at(MESSAGE_PAYLOAD_START),
                (float) message.at(MESSAGE_PAYLOAD_START + 1)
            );
        }
    }
    else if (message_type == MessageType::NEW_TRAJECTORY)
    {
        if (message.size() - MESSAGE_PAYLOAD_START >= 2)
        {
            // Message should be size >= 2
            int size_of_trajectory = (int)message.at(MESSAGE_PAYLOAD_START);
            float duration_of_trajectory = (float)message.at(MESSAGE_PAYLOAD_START + 1);

            int expected_size = size_of_trajectory * 5;

            // Payload size is initial size - header - 2 first floats
            int trajectory_payload_start = MESSAGE_PAYLOAD_START + 2;

            if (expected_size != (message.size() - trajectory_payload_start))
            {
                std::vector<TrajectoryPoint > trajectoryPoints;

                for (int i = 0; i < size_of_trajectory; i++)
                {
                    TrajectoryPoint tp;
                    tp.position.x = message.at(trajectory_payload_start + 5*i);
                    tp.position.y = message.at(trajectory_payload_start + 5*i + 1);
                    tp.position.theta = message.at(trajectory_payload_start + 5*i + 2);
                    tp.linearVelocity = message.at(trajectory_payload_start + 5*i + 3);
                    tp.angularVelocity = message.at(trajectory_payload_start + 5*i + 4);
                    trajectoryPoints.push_back(tp);
                }

                TrajectoryConfig tc;
                std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));
                TrajectoryVector newTrajectory;
                newTrajectory.push_back(traj);

                return std::make_shared<NewTrajectoryMessage >(
                    newTrajectory
                );
            }
        }
    }
    return std::make_shared<ErrorMessage >();
}

// function definitions
VecFloat ConfigurationMessage::serialize()
{
    return VecFloat();
}

VecFloat NewTrajectoryMessage::serialize()
{
    return VecFloat();
}

VecFloat MatchStateMessage::serialize()
{
    return VecFloat();
}

VecFloat ErrorMessage::serialize()
{
    return VecFloat();
}