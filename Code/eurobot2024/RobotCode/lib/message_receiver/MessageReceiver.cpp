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
    buffer = new char[SIZE_OF_BUFFER]();

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

    // listening to the assigned socket 
    listen(serverSocket, 5); 
}

MessageReceiver::~MessageReceiver()
{
    // closing the socket. 
    close(serverSocket); 
}

std::shared_ptr<Message > MessageReceiver::receive()
{
    sockaddr_in client_addr;
    socklen_t sin_size;
    // accepting connection request 
    int clientSocket 
        = accept(serverSocket, (struct sockaddr *)&client_addr, &sin_size); 

    Serial.print("Client socket: ");
    Serial.println(clientSocket);
    Serial.print("Accepted connection from: ");
    Serial.println(inet_ntoa(client_addr.sin_addr));
    Serial.print("Sender ID is: ");
    unsigned char *ip = (unsigned char*)&(client_addr.sin_addr);
    uint8_t senderId = ip[3];
    Serial.println(senderId);

    // recieving data 
    int sizeofreceiveddata;

    int len = 0;
    while((sizeofreceiveddata = recv(clientSocket, (uint8_t *)&(buffer[len]), SIZE_OF_BUFFER-len, 0)) > 0)
    {
        len += sizeofreceiveddata;
    }

    // end the connection to the client
    close(clientSocket);

    // parse the message
    Serial.print("Message length: ");
    Serial.println(len);
    std::shared_ptr<Message > message(Message::parse((uint8_t*) buffer, len, senderId));

    return message;
};