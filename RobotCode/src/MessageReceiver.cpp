#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 

#include <SampledTrajectory.h>

#define SIZE_OF_BUFFER 512

MessageReceiver::MessageReceiver()
{

};

void MessageReceiver::begin()
{
    buffer = new float[SIZE_OF_BUFFER]();

    // creating socket 
    serverSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // specifying the address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(778); 
    serverAddress.sin_addr.s_addr = INADDR_ANY; 
  
    // binding socket. 
    bind(serverSocket, (struct sockaddr*)&serverAddress, 
         sizeof(serverAddress)); 
}

MessageReceiver::~MessageReceiver()
{
    // closing the socket. 
    close(serverSocket); 
}

MessageType MessageReceiver::receive()
{
  
    // listening to the assigned socket 
    listen(serverSocket, 5); 
  
    // accepting connection request 
    int clientSocket 
        = accept(serverSocket, nullptr, nullptr); 
  
    // recieving data 
    int sizeofreceiveddata;
    tmpvec.clear();

    while((sizeofreceiveddata = recv(clientSocket, buffer, SIZE_OF_BUFFER*4, 0)) > 0)
    {
        for (int i = 0; i < sizeofreceiveddata / 4; i++)
        {
            float f = buffer[i];
            tmpvec.push_back(f);
        }
    }

    MessageType mt(MessageType::ERROR);
    
    float message_type = tmpvec.at(0);
    Serial.print("Received message type ");
    Serial.println(message_type);

    if (message_type == 0)
    {
        mt = MessageType::NEW_TRAJECTORY;

        int size_of_trajectory = tmpvec.at(1);
        float duration_of_trajectory = tmpvec.at(2);

        int expected_size = size_of_trajectory * 5 + 3;

        if (expected_size != tmpvec.size())
        {
            Serial.println("Decrepency in message sizes!");
            Serial.print("Expected ");
            Serial.print(expected_size);
            Serial.print(" received ");
            Serial.println(tmpvec.size());
            return MessageType::ERROR;
        }
        std::vector<TrajectoryPoint > trajectoryPoints;

        for (int i = 0; i < size_of_trajectory; i++)
        {
            TrajectoryPoint tp;
            tp.position.x = tmpvec.at(3 + 5*i);
            tp.position.y = tmpvec.at(3 + 5*i + 1);
            tp.position.theta = tmpvec.at(3 + 5*i + 2);
            tp.linearVelocity = tmpvec.at(3 + 5*i + 3);
            tp.angularVelocity = tmpvec.at(3 + 5*i + 4);
            trajectoryPoints.push_back(tp);
        }

        TrajectoryConfig tc;

        std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));

        targetTrajectory.clear();
        targetTrajectory.push_back(traj);
        
        tmpvec.clear();
    }
    else if (message_type == 1)
    {
        mt = MessageType::SET_ID;

        int expected_size = 2;

        if (expected_size != tmpvec.size())
        {
            Serial.println("Decrepency in message sizes!");
            Serial.print("Expected ");
            Serial.print(expected_size);
            Serial.print(" received ");
            Serial.println(tmpvec.size());
            return MessageType::ERROR;
        }

        newID = (int)tmpvec.at(1);
    }

    return mt;
};