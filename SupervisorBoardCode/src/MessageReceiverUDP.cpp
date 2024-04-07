#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <memory>

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

std::shared_ptr<Message > MessageReceiverUDP::receive()
{
  
    // // listening to the assigned socket 
    // listen(serverSocket, 5); 
  
    // accepting connection request 
    // int clientSocket 
    //     = accept(serverSocket, nullptr, nullptr); 
  
    // recieving data 
    int sizeofreceiveddata;
    receivedTrajectory.clear();
    
    // sockaddr_in si_other;
    // unsigned slen=sizeof(sockaddr);

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

    std::shared_ptr<Message > message(Message::parse(receivedTrajectory));

    return message;
};