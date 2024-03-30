#ifndef _MESSAGE_HPP
#define _MESSAGE_HPP

#include <vector>
#include <Utilities.h>
#include <memory>

/*
Messages from SCD to PAMI can be of categories:
- configuration message: set side (right side, left side)
- immediate new trajectory: pami should execute traj immediatly (i.e. setup trajectory)
- match state message: whether match started and currrent match time
- alternative travel trajectory: replanified trajectory to objective 

Messages from PAMI to SCD can be of categories:
- match state message
*/

using namespace miam::trajectory;
typedef std::vector<float > VecFloat;

enum MessageType
{
    CONFIGURATION = 0,
    NEW_TRAJECTORY = 1,
    MATCH_STATE = 2,
    ERROR = 99
};

class Message
{
public:
    MessageType get_message_type() { return messageType_; }
    uint8_t get_sender_id() { return senderId_; }

    static std::shared_ptr<Message > parse(VecFloat message, uint8_t senderId = 0);
    virtual VecFloat serialize();

private:
    MessageType messageType_;
    uint8_t senderId_;
    Message(MessageType mt, uint8_t senderId = 0) : messageType_(mt), senderId_(senderId) {};

    friend class ConfigurationMessage;
    friend class MatchStateMessage;
    friend class NewTrajectoryMessage;
    friend class ErrorMessage;
};

/* Specialized messages */

// CONFIGURATION
class ConfigurationMessage : public Message
{
public:
    ConfigurationMessage(bool isPlayingRightSide) :
        Message(MessageType::CONFIGURATION),
        isPlayingRightSide_(isPlayingRightSide) {};
    
    VecFloat serialize();

    bool isPlayingRightSide_;
};

// NEW TRAJECTORY
class NewTrajectoryMessage : public Message
{
public:
    NewTrajectoryMessage(TrajectoryVector trajectory) :
        Message(MessageType::NEW_TRAJECTORY),
        newTrajectory_(trajectory) {};

    VecFloat serialize();
    
    TrajectoryVector newTrajectory_;
};

// MATCH_STATE
class MatchStateMessage : public Message
{
public:
    MatchStateMessage(bool matchStarted, float matchTime = 0.0) :
        Message(MessageType::MATCH_STATE),
        matchStarted_(matchStarted),
        matchTime_(matchTime) {};
    
    VecFloat serialize();

    bool matchStarted_;
    float matchTime_;
};

// ERROR
class ErrorMessage : public Message
{
public:
    ErrorMessage() : Message(MessageType::ERROR) {};  

    VecFloat serialize(); 
};

#endif