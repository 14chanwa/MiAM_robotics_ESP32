#ifndef _MESSAGE_HPP
#define _MESSAGE_HPP

#include <Utilities.h>
#include <memory>

/*
Messages from SCD to PAMI can be of categories:
- configuration message: set side (right side, left side)
- immediate new trajectory: pami should execute traj immediatly (i.e. setup trajectory)
- match state message: whether match started and currrent match time
- alternative travel trajectory: replanified trajectory to objective 

Messages from PAMI to SCD can be of categories:
- pami report message
*/

using namespace miam::trajectory;

enum MessageType
{
    CONFIGURATION = 0,
    NEW_TRAJECTORY = 1,
    MATCH_STATE = 2,
    PAMI_REPORT = 10,
    ERROR = 99
};

enum PlayingSide
{
    BLUE_SIDE = 0,
    YELLOW_SIDE = 1
};

class Message
{
public:
    MessageType get_message_type() { return messageType_; }
    uint8_t get_sender_id() { return senderId_; }

    static std::shared_ptr<Message > parse(const uint8_t* message, int sizeOfMessage, uint8_t senderId = 255);

    virtual int serialize(uint8_t* results, int maxsize) = 0;
    
    static uint get_expected_size() { return 2; }

private:
    MessageType messageType_;
    uint8_t senderId_;
    
    Message(MessageType mt, uint8_t senderId = 255) : messageType_(mt), senderId_(senderId) {};

    void write_header(uint8_t* results, uint& byte_index);
    void read_header(const uint8_t* buffer, uint& byte_index);

    friend class ConfigurationMessage;
    friend class MatchStateMessage;
    friend class NewTrajectoryMessage;
    friend class ErrorMessage;
    friend class PamiReportMessage;
};

/* Specialized messages */

// CONFIGURATION
class ConfigurationMessage : public Message
{
public:
    ConfigurationMessage(PlayingSide playingSide, bool stopMotors, uint8_t senderId = 255) :
        Message(MessageType::CONFIGURATION, senderId),
        playingSide_(playingSide),
        stopMotors_(stopMotors) {};
    ConfigurationMessage(const uint8_t* buffer, const uint size);
    
    int serialize(uint8_t* results, int maxsize);
    static uint get_expected_size() { return Message::get_expected_size() + 2; }

    PlayingSide playingSide_;
    bool stopMotors_;
};

// NEW TRAJECTORY
class NewTrajectoryMessage : public Message
{
public:
    NewTrajectoryMessage(TrajectoryVector trajectory, uint8_t senderId = 255) :
        Message(MessageType::NEW_TRAJECTORY, senderId),
        newTrajectory_(trajectory) {};
    NewTrajectoryMessage(const uint8_t* buffer, const uint size);

    int serialize(uint8_t* results, int maxsize);
    static uint get_expected_size() { return Message::get_expected_size() + 4 + 4 + 5*(1+1); /* Minimum 1 + 1 pts */ }
    
    TrajectoryVector newTrajectory_;
};

// MATCH_STATE
class MatchStateMessage : public Message
{
public:
    MatchStateMessage(bool matchStarted, float matchTime, uint8_t senderId = 255) :
        Message(MessageType::MATCH_STATE, senderId),
        matchStarted_(matchStarted),
        matchTime_(matchTime) {};
    MatchStateMessage(const uint8_t* buffer, const uint size);
    
    int serialize(uint8_t* results, int maxsize);
    static uint get_expected_size() { return Message::get_expected_size() + 1 + 4; }

    bool matchStarted_;
    float matchTime_;
};

// ERROR
class ErrorMessage : public Message
{
public:
    ErrorMessage(uint8_t senderId = 255) : Message(MessageType::ERROR, senderId) {};  

    int serialize(uint8_t* results, int maxsize);
};

// PAMI_REPORT
class PamiReportMessage : public Message
{
public:
    PamiReportMessage(
        bool matchStarted, 
        float matchTime, 
        PlayingSide playingSide, 
        float batteryReading,
        uint8_t senderId = 255
    ) : Message(MessageType::PAMI_REPORT, senderId),
        matchStarted_(matchStarted),
        matchTime_(matchTime),
        playingSide_(playingSide),
        batteryReading_(batteryReading) {};
    PamiReportMessage(const uint8_t* buffer, const uint size);
    
    int serialize(uint8_t* results, int maxsize);
    static uint get_expected_size() { return Message::get_expected_size() + 1 + 4 + 1 + 4; }

    bool matchStarted_;
    float matchTime_;
    PlayingSide playingSide_;
    float batteryReading_;
};

#endif