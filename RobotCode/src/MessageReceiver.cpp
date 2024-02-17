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
        // cout << "Receiving: " << sizeofreceiveddata << std::endl; 
        // cout << "Message from client: " << buffer 
        //         << endl; 
        for (int i = 0; i < sizeofreceiveddata / 4; i++)
        {
            float f = buffer[i];
            // cout << f << endl;
            tmpvec.push_back(f);
        }
    }

    // for (int i=0; i < tmpvec.size(); i++)
    // {
    //     std::cout << tmpvec.at(i);
    //     if (i > 0 && i % 5 == 0)
    //         std::cout << std::endl;
    // }
    

    int size_of_trajectory = tmpvec.at(0);
    float duration_of_trajectory = tmpvec.at(1);

    if ((size_of_trajectory * 5 + 2) != tmpvec.size())
    {
        Serial.println("Decrepency in received traj!");
        Serial.print("Expected ");
        Serial.print(size_of_trajectory * 5 + 2);
        Serial.print(" received ");
        Serial.println(tmpvec.size());
        return MessageType::ERROR;
    }

    // cout << "Size of trajectory: " << size_of_trajectory << endl;
    // cout << "Duration of trajectory: " << duration_of_trajectory << endl;

    std::vector<TrajectoryPoint > trajectoryPoints;
    // int serializationIndex = 2;
    for (int i = 0; i < size_of_trajectory; i++)
    {
        TrajectoryPoint tp;
        tp.position.x = tmpvec.at(2 + 5*i);
        tp.position.y = tmpvec.at(2 + 5*i + 1);
        tp.position.theta = tmpvec.at(2 + 5*i + 2);
        tp.linearVelocity = tmpvec.at(2 + 5*i + 3);
        tp.angularVelocity = tmpvec.at(2 + 5*i + 4);
        trajectoryPoints.push_back(tp);
    }

    TrajectoryConfig tc;

    std::shared_ptr<SampledTrajectory > traj(new SampledTrajectory(tc, trajectoryPoints, duration_of_trajectory));

    targetTrajectory.clear();
    targetTrajectory.push_back(traj);

    // for (auto& tp : trajectoryPoints)
    // {
    //     std::cout << tp << std::endl;
    // }
    
    tmpvec.clear();

    return NEW_TRAJECTORY_RECEIVED;
};