#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>
#include <Message.hpp>

// #define USE_WIFICLIENT_API
#define USE_ASYNCTCP

#ifdef USE_WIFICLIENT_API
    #include <WiFi.h>
#else 
#ifdef USE_ASYNCTCP
#include <Arduino.h>
#include <AsyncTCP.h>
#endif
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
#ifdef USE_WIFICLIENT_API
    WiFiServer* server;
#else
#ifdef USE_ASYNCTCP
    AsyncServer* server;
#else
    int serverSocket;
    int clientSocket;
#endif
#endif

    std::vector<float > receivedTrajectory;
};

#endif