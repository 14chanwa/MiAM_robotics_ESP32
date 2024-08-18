#include <Message.hpp>
#include <SampledTrajectory.h>
#include <cmath>

#define DEBUG_MESSAGE
#ifdef DEBUG_MESSAGE
#include <Arduino.h>
#endif

#define MESSAGE_PAYLOAD_START 1

#define TRAJECTORY_SERIALIZATION_DELTAT 0.1

using namespace miam::trajectory;

std::shared_ptr<Message > Message::parse(float* message, int sizeOfMessage, uint8_t senderId)
{
    if (sizeOfMessage == 0)
    {
        return std::make_shared<ErrorMessage >(senderId);;
    }

    // Message type is the first float casted to int
    int message_type = (int)message[0];

    // // Sender is the second float casted to int
    // int sender_id = (int) message.at(1);

    if (message_type == MessageType::CONFIGURATION)
    {
        // Check size of payload
        if (sizeOfMessage - MESSAGE_PAYLOAD_START == 2)
        {
            // First byte of payload is configuration
            if ((bool)message[MESSAGE_PAYLOAD_START] == PlayingSide::BLUE_SIDE)
            {
                return std::make_shared<ConfigurationMessage >(
                    PlayingSide::BLUE_SIDE,
                    (bool) message[MESSAGE_PAYLOAD_START + 1],
                    senderId
                );
            }
            else if ((bool)message[MESSAGE_PAYLOAD_START] == PlayingSide::YELLOW_SIDE)
            {
                return std::make_shared<ConfigurationMessage >(
                    PlayingSide::YELLOW_SIDE,
                    (bool) message[MESSAGE_PAYLOAD_START + 1],
                    senderId
                );
            }
        }
    }
    else if (message_type == MessageType::MATCH_STATE)
    {
        // Check size of payload
        if (sizeOfMessage - MESSAGE_PAYLOAD_START == 2)
        {
            // First byte of payload is match started
            // Second byte of payload is match time
            return std::make_shared<MatchStateMessage >(
                (bool) message[MESSAGE_PAYLOAD_START],
                (float) message[MESSAGE_PAYLOAD_START + 1],
                senderId
            );
        }
    }
    else if (message_type == MessageType::NEW_TRAJECTORY)
    {
        if (sizeOfMessage - MESSAGE_PAYLOAD_START >= 2)
        {
            // Message should be size >= 2
            int size_of_trajectory = (int)message[MESSAGE_PAYLOAD_START];
            float duration_of_trajectory = (float)message[MESSAGE_PAYLOAD_START + 1];

            int expected_size = size_of_trajectory * 5;

            // Payload size is initial size - header - 2 first floats
            int trajectory_payload_start = MESSAGE_PAYLOAD_START + 2;

            if (expected_size == (sizeOfMessage - trajectory_payload_start))
            {
                std::vector<TrajectoryPoint > trajectoryPoints;

                for (int i = 0; i < size_of_trajectory; i++)
                {
                    TrajectoryPoint tp;
                    tp.position.x = message[trajectory_payload_start + 5*i];
                    tp.position.y = message[trajectory_payload_start + 5*i + 1];
                    tp.position.theta = message[trajectory_payload_start + 5*i + 2];
                    tp.linearVelocity = message[trajectory_payload_start + 5*i + 3];
                    tp.angularVelocity = message[trajectory_payload_start + 5*i + 4];
                    trajectoryPoints.push_back(tp);
                }

                TrajectoryConfig tc;
                std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));
                TrajectoryVector newTrajectory;
                newTrajectory.push_back(traj);

                return std::make_shared<NewTrajectoryMessage >(
                    newTrajectory,
                    senderId
                );
            }
        }
    }
    else if (message_type == MessageType::PAMI_REPORT)
    {
        // Check size of payload
        if (sizeOfMessage - MESSAGE_PAYLOAD_START == 4)
        {
            // First byte of payload is match started
            // Second byte of payload is match time
            // 3rd byte is side
            if ((bool)message[MESSAGE_PAYLOAD_START+2] == PlayingSide::BLUE_SIDE)
            {
                return std::make_shared<PamiReportMessage >(
                    (bool) message[MESSAGE_PAYLOAD_START],
                    (float) message[MESSAGE_PAYLOAD_START + 1],
                    PlayingSide::BLUE_SIDE,
                    message[MESSAGE_PAYLOAD_START + 3],
                    senderId
                );
            }
            else if ((bool)message[MESSAGE_PAYLOAD_START+2] == PlayingSide::YELLOW_SIDE)
            {
                // First byte of payload is match started
                // Second byte of payload is match time
                return std::make_shared<PamiReportMessage >(
                    (bool) message[MESSAGE_PAYLOAD_START],
                    (float) message[MESSAGE_PAYLOAD_START + 1],
                    PlayingSide::YELLOW_SIDE,
                    message[MESSAGE_PAYLOAD_START + 3],
                    senderId
                );
            }
        }
    }
#ifdef DEBUG_MESSAGE
    Serial.print("Parsed error from ");
    Serial.println(senderId);
#endif
    return std::make_shared<ErrorMessage >(senderId);
}

// function definitions
int  ConfigurationMessage::serialize(float* results, int maxsize)
{
    // Total size in floats is header size =3
    if (maxsize < 3) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    // Second byte is side
    results[current_index++] = (float)playingSide_;
    // Third byte is stop motors
    results[current_index++] = (float)stopMotors_;
    return current_index;
}

int NewTrajectoryMessage::serialize(float* results, int maxsize)
{
    // Serializing the trajectory:
    // N+1 = Number of points is duration / TRAJECTORY_SERIALIZATION_DELTAT + 1
    // N = number of time intervals
    int N = std::ceil(newTrajectory_.getDuration() / TRAJECTORY_SERIALIZATION_DELTAT);
    int deltat = newTrajectory_.getDuration() / N;

    // Total size in floats is header size + (N+1*5) pts
    if (maxsize < 3 + N+1) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    // Second byte is size of trajectory in number of points
    results[current_index++] = (float)(N+1);
    // Third byte is duration
    results[current_index++] = (float)newTrajectory_.getDuration();
    // Following bytes are trajectory points
    for (int i = 0; i < N+1; i++)
    {
        TrajectoryPoint pt;
        if (i < N)
        {
            pt = newTrajectory_.getCurrentPoint(deltat * i);
        } else {
            pt = newTrajectory_.getEndPoint();
        }
        results[current_index++] = (float)pt.position.x;
        results[current_index++] = (float)pt.position.y;
        results[current_index++] = (float)pt.position.theta;
        results[current_index++] = (float)pt.linearVelocity;
        results[current_index++] = (float)pt.angularVelocity;
    }
    return current_index;
}

int MatchStateMessage::serialize(float* results, int maxsize)
{
    // Total size in floats is header size =3
    if (maxsize < 3) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    // Second byte is match started
    results[current_index++] = (float)matchStarted_;
    // Third byte is match time
    results[current_index++] = (float)matchTime_;
    return current_index;
}

int ErrorMessage::serialize(float* results, int maxsize)
{
    // Total size in floats is header size =1
    if (maxsize < 1) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    return current_index;
}

int PamiReportMessage::serialize(float* results, int maxsize)
{
    // Total size in floats is header size =4
    if (maxsize < 5) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    // Second byte is match started
    results[current_index++] = (float)matchStarted_;
    // Third byte is match time
    results[current_index++] = (float)matchTime_;
    // 4th byte is playing side
    results[current_index++] = (float)playingSide_;
    // 5th byte is battery reading
    results[current_index++] = (float)batteryReading_;
    return current_index;
}