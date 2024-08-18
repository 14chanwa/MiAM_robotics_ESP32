#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <Utilities.h>
#include <Message.hpp>

using namespace miam::trajectory;

namespace MessageReceiver
{
    void startListening();
};

#endif