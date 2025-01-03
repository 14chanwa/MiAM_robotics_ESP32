#include <Message.hpp>
#include <SampledTrajectory.h>
#include <cmath>

#define DEBUG_MESSAGE
#ifdef DEBUG_MESSAGE
#include <Arduino.h>
#endif

#define MESSAGE_PAYLOAD_START 2

#define TRAJECTORY_SERIALIZATION_DELTAT 0.1

using namespace miam::trajectory;

// read a T and increment byte counter
template <class T> T read_from_buffer(const uint8_t* buffer, uint& byte_index)
{
    T res = buffer[byte_index];
    byte_index += sizeof(T);
    return res;
}

// write a T and increment byte counter
template <class T> void write_to_buffer(uint8_t* buffer, const T target, uint& byte_index)
{
    memcpy(&(buffer[byte_index]), &target, sizeof(T));
    byte_index += sizeof(T);
}

void Message::write_header(uint8_t* results, uint& byte_index)
{
    // Message type
    write_to_buffer<uint8_t>(results, (uint8_t)messageType_, byte_index);
    // Sender id
    write_to_buffer<uint8_t>(results, (uint8_t)senderId_, byte_index);
}

std::shared_ptr<Message > Message::parse(const uint8_t* message, int sizeOfMessage, uint8_t senderId)
{
    if (sizeOfMessage == 0)
    {
        return std::make_shared<ErrorMessage >(senderId);;
    }

    Serial.print("Message is: ");
    for (uint i=0; i<sizeOfMessage; i++)
    {
        Serial.print(message[i]);
        Serial.print(" ");
    }
    Serial.println();

    uint byte_index = 0;

    // Message type is the first byte casted to int
    uint8_t message_type = read_from_buffer<uint8_t>(message, byte_index);

    // Sender is the second byte casted to int
    senderId = read_from_buffer<uint8_t>(message, byte_index);

    if (message_type == MessageType::CONFIGURATION)
    {
        // Check size of payload
        if (sizeOfMessage == ConfigurationMessage::get_expected_size())
        {
            return std::make_shared<ConfigurationMessage >(message, sizeOfMessage);
        }
    }
    else if (message_type == MessageType::MATCH_STATE)
    {
        // Check size of payload
        if (sizeOfMessage == MatchStateMessage::get_expected_size())
        {
            return std::make_shared<MatchStateMessage >(message, sizeOfMessage);
        }
    }
    else if (message_type == MessageType::NEW_TRAJECTORY)
    {
        if (sizeOfMessage >= NewTrajectoryMessage::get_expected_size())
        {
            return std::make_shared<NewTrajectoryMessage >(message, sizeOfMessage);
        }
    }
    else if (message_type == MessageType::PAMI_REPORT)
    {
        // Check size of payload
        if (sizeOfMessage == PamiReportMessage::get_expected_size())
        {
            return std::make_shared<PamiReportMessage >(message, sizeOfMessage);
        }
    }
#ifdef DEBUG_MESSAGE
    Serial.print("Parsed error from ");
    Serial.println(senderId);
#endif
    return std::make_shared<ErrorMessage >(senderId);
}

/* 
 * Message 
 */

void Message::read_header(const uint8_t* buffer, uint& byte_index)
{
    messageType_ = (MessageType) read_from_buffer<uint8_t >(buffer, byte_index);
    senderId_ = read_from_buffer<uint8_t >(buffer, byte_index);
}

/* 
 * ConfigurationMessage 
 */

ConfigurationMessage::ConfigurationMessage(const uint8_t* buffer, const uint size) : Message(MessageType::ERROR)
{
    uint byte_index = 0;
    Message::read_header(buffer, byte_index);

    // Read payload
    if ((bool)read_from_buffer<uint8_t >(buffer, byte_index) == PlayingSide::YELLOW_SIDE)
    {
        playingSide_ = PlayingSide::YELLOW_SIDE;
    }
    else
    {
        playingSide_ = PlayingSide::BLUE_SIDE;
    }
    stopMotors_ = (bool)read_from_buffer<uint8_t >(buffer, byte_index);
}

int ConfigurationMessage::serialize(uint8_t* results, int maxsize)
{
    if (maxsize < get_expected_size()) return -1;

    uint byte_index = 0;
    Message::write_header(results, byte_index);

    // Side -> byte
    write_to_buffer<uint8_t>(results, (uint8_t)playingSide_, byte_index);
    // stop motors -> byte
    write_to_buffer<uint8_t>(results, (uint8_t)stopMotors_, byte_index);

    return byte_index;
}

/* 
 * NewTrajectoryMessage 
 */

NewTrajectoryMessage::NewTrajectoryMessage(const uint8_t* buffer, const uint size) : Message(MessageType::ERROR)
{
    uint byte_index = 0;
    Message::read_header(buffer, byte_index);

    // Read payload
    int size_of_trajectory = read_from_buffer<int>(buffer, byte_index);
    float duration_of_trajectory = read_from_buffer<float>(buffer, byte_index);
    int expected_size = size_of_trajectory * 5;

    // TODO add size check

    std::vector<TrajectoryPoint > trajectoryPoints;
    for (int i = 0; i < size_of_trajectory; i++)
    {
        TrajectoryPoint tp;
        tp.position.x = read_from_buffer<float>(buffer, byte_index);
        tp.position.y = read_from_buffer<float>(buffer, byte_index);
        tp.position.theta = read_from_buffer<float>(buffer, byte_index);
        tp.linearVelocity = read_from_buffer<float>(buffer, byte_index);
        tp.angularVelocity = read_from_buffer<float>(buffer, byte_index);
        trajectoryPoints.push_back(tp);
    }
    TrajectoryConfig tc;
    std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));
    newTrajectory_.push_back(traj);
}

int NewTrajectoryMessage::serialize(uint8_t* results, int maxsize)
{
    if (maxsize < get_expected_size()) return -1;

    // Serializing the trajectory:
    // N+1 = Number of points is duration / TRAJECTORY_SERIALIZATION_DELTAT + 1
    // N = number of time intervals
    int N = std::ceil(newTrajectory_.getDuration() / TRAJECTORY_SERIALIZATION_DELTAT);
    int deltat = newTrajectory_.getDuration() / N;

    uint byte_index = 0;
    // size of trajectory in number of points -> int
    write_to_buffer<int>(results, (int)(N+1), byte_index);
    // duration -> float
    write_to_buffer<float>(results, (float)newTrajectory_.getDuration(), byte_index);
    // Following bytes are trajectory points -> float
    for (int i = 0; i < N+1; i++)
    {
        TrajectoryPoint pt;
        if (i < N)
        {
            pt = newTrajectory_.getCurrentPoint(deltat * i);
        } else {
            pt = newTrajectory_.getEndPoint();
        }

        write_to_buffer<float>(results, (float)pt.position.x, byte_index);
        write_to_buffer<float>(results, (float)pt.position.y, byte_index);
        write_to_buffer<float>(results, (float)pt.position.theta, byte_index);
        write_to_buffer<float>(results, (float)pt.linearVelocity, byte_index);
        write_to_buffer<float>(results, (float)pt.angularVelocity, byte_index);
    }
    return byte_index;
}

/* 
 * MatchStateMessage 
 */

MatchStateMessage::MatchStateMessage(const uint8_t* buffer, const uint size) : Message(MessageType::ERROR)
{
    uint byte_index = 0;
    Message::read_header(buffer, byte_index);

    // Read payload
    matchStarted_ = (bool)read_from_buffer<uint8_t >(buffer, byte_index);
    matchTime_ = read_from_buffer<float >(buffer, byte_index);
}

int MatchStateMessage::serialize(uint8_t* results, int maxsize)
{
    if (maxsize < get_expected_size()) return -1;

    uint byte_index = 0;
    Message::write_header(results, byte_index);

    // match started -> byte
    write_to_buffer<uint8_t>(results, (uint8_t)matchStarted_, byte_index);
    // match time -> float
    write_to_buffer<float>(results, matchTime_, byte_index);

    return byte_index;
}

/* 
 * ErrorMessage 
 */

int ErrorMessage::serialize(uint8_t* results, int maxsize)
{
    // Total size in floats is header size =1
    if (maxsize < 1) return -1;

    int current_index = 0;
    // First byte is message type
    results[current_index++] = (float)get_message_type();
    return current_index;
}

/* 
 * PamiReportMessage 
 */

PamiReportMessage::PamiReportMessage(const uint8_t* buffer, const uint size) : Message(MessageType::ERROR)
{
    uint byte_index = 0;
    Message::read_header(buffer, byte_index);

    // Read payload
    matchStarted_ = (bool)read_from_buffer<uint8_t >(buffer, byte_index);
    matchTime_ = read_from_buffer<float >(buffer, byte_index);
    if ((bool)read_from_buffer<uint8_t >(buffer, byte_index) == PlayingSide::YELLOW_SIDE)
    {
        playingSide_ = PlayingSide::YELLOW_SIDE;
    }
    else
    {
        playingSide_ = PlayingSide::BLUE_SIDE;
    }
    batteryReading_ = read_from_buffer<float >(buffer, byte_index);
}

int PamiReportMessage::serialize(uint8_t* results, int maxsize)
{
    if (maxsize < get_expected_size()) return -1;

    uint byte_index = 0;
    Message::write_header(results, byte_index);

    // match started -> byte
    write_to_buffer<uint8_t>(results, (uint8_t)matchStarted_, byte_index);
    // match time -> float
    write_to_buffer<float>(results, matchTime_, byte_index);
    // playing side -> uint8_t
    write_to_buffer<uint8_t>(results, (uint8_t)playingSide_, byte_index);
    // battery reading -> float
    write_to_buffer<float>(results, batteryReading_, byte_index);

    return byte_index;
}