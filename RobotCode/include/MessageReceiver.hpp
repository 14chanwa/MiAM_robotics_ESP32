#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <Utilities.h>

using namespace miam::trajectory;

enum MessageType
{
    NEW_TRAJECTORY = 0,
    SET_ID = 1,
    ERROR = 99
};

class MessageReceiver
{

public:
    MessageReceiver();
    ~MessageReceiver();
    void begin();
    
    MessageType receive();

    TrajectoryVector targetTrajectory;
    int newID;

private:
    float* buffer; 
    std::vector<float > tmpvec;
    int serverSocket;
};

#endif