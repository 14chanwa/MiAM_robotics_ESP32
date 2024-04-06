#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 

#include <SampledTrajectory.h>

#include <Arduino.h>


// #define USE_WIFICLIENT_API

#ifdef USE_WIFICLIENT_API
#include <WiFi.h>

WiFiServer server(778);
WiFiClient client = server.available();
#endif

#define SIZE_OF_BUFFER 512



MessageReceiver::MessageReceiver()
{

};

void MessageReceiver::begin()
{
    buffer = new float[SIZE_OF_BUFFER*4]();

#ifdef USE_WIFICLIENT_API
    server.begin();
#else
    // creating socket 
    serverSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // specifying the address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(778); 
    serverAddress.sin_addr.s_addr = INADDR_ANY; 
  
    struct timeval timeout;      
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    
    if (setsockopt (serverSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                sizeof timeout) < 0)
        Serial.println("setsockopt failed");

    // binding socket. 
    bind(serverSocket, (struct sockaddr*)&serverAddress, 
         sizeof(serverAddress)); 

    // listening to the assigned socket 
    listen(serverSocket, 5); 
#endif
}

MessageReceiver::~MessageReceiver()
{
#ifndef USE_WIFICLIENT_API
    // closing the socket. 
    close(serverSocket); 
#endif
}

std::shared_ptr<Message > MessageReceiver::receive()
{
#ifdef USE_WIFICLIENT_API
    while (client.connected()) {            // loop while the client's connected

      if (client.available()) {             // if there's bytes to read from the client,

        char c = client.read();

      }
    }
#else
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
    receivedTrajectory.clear();

    while((sizeofreceiveddata = recv(clientSocket, buffer, SIZE_OF_BUFFER*4, 0)) > 0)
    {
        for (int i = 0; i < sizeofreceiveddata / 4; i++)
        {
            float f = buffer[i];
            receivedTrajectory.push_back(f);
        }
    }

    // end the connection to the client
    close(clientSocket);
#endif

    // parse the message
    Serial.print("Message length: ");
    Serial.println(receivedTrajectory.size());
    for (auto f : receivedTrajectory)
    {
        Serial.print(f);
        Serial.print(" ");
    }
    Serial.println();

    std::shared_ptr<Message > message(Message::parse(receivedTrajectory, senderId));

    return message;

};