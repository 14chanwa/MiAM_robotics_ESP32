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

private:
    char* buffer; 
    int serverSocket;
    int clientSocket;
};

#endif