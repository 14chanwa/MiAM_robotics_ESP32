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
    
    std::shared_ptr<Message > receive();

    // static void stopReceiving();

private:
    char* buffer; 
    char* sendBuffer;
    int serverSocket;
    int clientSocket;
    std::vector<float > receivedTrajectory;
};


class MessageReceiverUDP
{

public:
    MessageReceiverUDP();
    ~MessageReceiverUDP();
    void begin();
    
    std::shared_ptr<Message > receive();

private:
    float* buffer; 
    int serverSocket;
    int clientSocket;
    std::vector<float > receivedTrajectory;
};

#endif