#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>
#include <Message.hpp>

#define USE_WIFICLIENT_API

#ifdef USE_WIFICLIENT_API
#include <WiFi.h>
#endif


using namespace miam::trajectory;


class MessageReceiver
{

public:
    MessageReceiver();
    ~MessageReceiver();
    void begin();
    
    std::shared_ptr<Message > receive();

    static void stopReceiving();

private:
    char* buffer; 
    char* sendBuffer;
#ifdef USE_WIFICLIENT_API
    WiFiServer* server;
#else
    int serverSocket;
    int clientSocket;
#endif

    std::vector<float > receivedTrajectory;
};


// class MessageReceiverUDP
// {

// public:
//     MessageReceiverUDP();
//     ~MessageReceiverUDP();
//     void begin();
    
//     std::shared_ptr<Message > receive();

// private:
//     float* buffer; 
//     int serverSocket;
//     int clientSocket;
//     std::vector<float > receivedTrajectory;
// };

#endif