#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>
#include <Message.hpp>

using namespace miam::trajectory;


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