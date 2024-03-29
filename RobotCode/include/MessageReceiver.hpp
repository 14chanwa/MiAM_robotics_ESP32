#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>

using namespace miam::trajectory;

enum MessageType
{
    NEW_TRAJECTORY = 0,
    SET_ID = 1,
    NEW_TRAJECTORY_SAVE = 2,
    MATCH_STATE = 3,
    ERROR = 99
};


class MessageReceiver
{

public:
    MessageReceiver();
    ~MessageReceiver();
    void begin();
    
    MessageType receive();

    std::vector<float > receivedTrajectory;
    TrajectoryVector targetTrajectory;
    int newID;

    bool matchStarted;
    float matchCurrentTime;

private:
    float* buffer; 
    int serverSocket;
    int clientSocket;
};


class MessageReceiverUDP
{

public:
    MessageReceiverUDP();
    ~MessageReceiverUDP();
    void begin();
    
    MessageType receive();

    std::vector<float > receivedTrajectory;
    TrajectoryVector targetTrajectory;
    int newID;

    bool matchStarted;
    float matchCurrentTime;

private:
    float* buffer; 
    int serverSocket;
    int clientSocket;
};

#endif