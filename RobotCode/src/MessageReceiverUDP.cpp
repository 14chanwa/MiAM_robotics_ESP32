#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 

#include <SampledTrajectory.h>

#define SIZE_OF_BUFFER 512

MessageReceiverUDP::MessageReceiverUDP()
{

};

void MessageReceiverUDP::begin()
{
    buffer = new float[SIZE_OF_BUFFER]();

    // creating socket 
    clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 

    int broadcast=1;
    setsockopt(clientSocket, SOL_SOCKET, SO_BROADCAST,
                &broadcast, sizeof broadcast);
  
    // specifying the address 
    sockaddr_in si_me; 
    memset(&si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET; 
    si_me.sin_port = htons(779); 
    si_me.sin_addr.s_addr = INADDR_ANY; 
  
    // binding socket. 
    bind(clientSocket, (struct sockaddr*)&si_me, 
         sizeof(sockaddr)); 
}

MessageReceiverUDP::~MessageReceiverUDP()
{
    // closing the socket. 
    close(clientSocket); 
}

MessageType MessageReceiverUDP::receive()
{
  
    // // listening to the assigned socket 
    // listen(serverSocket, 5); 
  
    // accepting connection request 
    // int clientSocket 
    //     = accept(serverSocket, nullptr, nullptr); 
  
    // recieving data 
    int sizeofreceiveddata;
    receivedTrajectory.clear();
    
    sockaddr_in si_other;
    unsigned slen=sizeof(sockaddr);

    if((sizeofreceiveddata = recv(clientSocket, buffer, SIZE_OF_BUFFER*4, 0)) > 0)
    // while((sizeofreceiveddata = recvfrom(clientSocket, buffer, SIZE_OF_BUFFER*4, 0, (sockaddr *)&si_other, &slen)) > 0)
    {
        Serial.println("Receiving");
        for (int i = 0; i < sizeofreceiveddata / 4; i++)
        {
            float f = buffer[i];
            receivedTrajectory.push_back(f);
        }
    }

    MessageType mt(MessageType::ERROR);
    
    float message_type = receivedTrajectory.at(0);
    Serial.print("Received message type ");
    Serial.println(message_type);

    if (message_type == 0)
    {
        mt = MessageType::NEW_TRAJECTORY;

        int size_of_trajectory = receivedTrajectory.at(1);
        float duration_of_trajectory = receivedTrajectory.at(2);

        int expected_size = size_of_trajectory * 5 + 3;

        if (expected_size != receivedTrajectory.size())
        {
            Serial.println("Decrepency in message sizes!");
            Serial.print("Expected ");
            Serial.print(expected_size);
            Serial.print(" received ");
            Serial.println(receivedTrajectory.size());
            return MessageType::ERROR;
        }
        std::vector<TrajectoryPoint > trajectoryPoints;

        for (int i = 0; i < size_of_trajectory; i++)
        {
            TrajectoryPoint tp;
            tp.position.x = receivedTrajectory.at(3 + 5*i);
            tp.position.y = receivedTrajectory.at(3 + 5*i + 1);
            tp.position.theta = receivedTrajectory.at(3 + 5*i + 2);
            tp.linearVelocity = receivedTrajectory.at(3 + 5*i + 3);
            tp.angularVelocity = receivedTrajectory.at(3 + 5*i + 4);
            trajectoryPoints.push_back(tp);
        }

        TrajectoryConfig tc;

        std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));

        targetTrajectory.clear();
        targetTrajectory.push_back(traj);
        
        receivedTrajectory.clear();
    }
    // else if (message_type == 1)
    // {
    //     mt = MessageType::SET_ID;

    //     int expected_size = 2;

    //     if (expected_size != receivedTrajectory.size())
    //     {
    //         Serial.println("Decrepency in message sizes!");
    //         Serial.print("Expected ");
    //         Serial.print(expected_size);
    //         Serial.print(" received ");
    //         Serial.println(receivedTrajectory.size());
    //         return MessageType::ERROR;
    //     }

    //     newID = (int)receivedTrajectory.at(1);
    // }
    // else if (message_type == 2)
    // {
    //     mt = MessageType::NEW_TRAJECTORY_SAVE;

    //     int size_of_trajectory = receivedTrajectory.at(1);
    //     // float duration_of_trajectory = receivedTrajectory.at(2);

    //     int expected_size = size_of_trajectory * 5 + 3;

    //     if (expected_size != receivedTrajectory.size())
    //     {
    //         Serial.println("Decrepency in message sizes!");
    //         Serial.print("Expected ");
    //         Serial.print(expected_size);
    //         Serial.print(" received ");
    //         Serial.println(receivedTrajectory.size());
    //         return MessageType::ERROR;
    //     }        
    // }
    else if (message_type == 3)
    {
        mt = MessageType::MATCH_STATE;
        matchStarted = (bool)receivedTrajectory.at(1);
        matchCurrentTime = (float)receivedTrajectory.at(2);
    }

    return mt;
};