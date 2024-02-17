#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>

using namespace miam::trajectory;

enum MessageType
{
    NEW_TRAJECTORY_RECEIVED,
    ERROR
};

class MessageReceiver
{

public:
    MessageReceiver();
    MessageType receive();

    TrajectoryVector targetTrajectory;

};

#endif