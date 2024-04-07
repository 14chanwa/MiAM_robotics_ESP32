#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <SampledTrajectory.h>
#include <Match.hpp>

#include <Arduino.h>

#define DEBUG
// #define USE_TIMEOUT


#define SIZE_OF_BUFFER 512
bool stopReceiving_ = false;

MessageReceiver::MessageReceiver(){
#ifdef USE_WIFICLIENT_API
    server = new WiFiServer(778);
#endif
};

void MessageReceiver::begin()
{
    buffer = new char[SIZE_OF_BUFFER]();
    sendBuffer = new char[SIZE_OF_BUFFER]();

#ifdef USE_WIFICLIENT_API
    server->begin();
    server->setTimeout(1);
#else
    // creating socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    // specifying the address
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(778);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

#ifdef USE_TIMEOUT
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    if (setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                   sizeof timeout) < 0)
        Serial.println("setsockopt failed");
#endif

    // binding socket.
    bind(serverSocket, (struct sockaddr *)&serverAddress,
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

std::shared_ptr<Message> MessageReceiver::receive()
{

    std::shared_ptr<Message> message = nullptr;

    if (stopReceiving_)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        return message;
    }
    WiFiClient client = server->available();
    client.setTimeout(1);

#ifdef USE_WIFICLIENT_API
    if (client)
    { // loop while the client's connected
        in_addr adress;
        IPAddress remoteIP = client.remoteIP();
        inet_aton(remoteIP.toString().c_str(), &adress);
        unsigned char *ip = (unsigned char *)&(adress.s_addr);
        uint8_t senderId = ip[3];
#ifdef DEBUG
        Serial.print(">>>> Client is connected: ");
        Serial.println(client.remoteIP());
#endif
        // receive data
        receivedTrajectory.clear();

        while (client.connected() && client.available())
        {
            int len = client.read((uint8_t *)buffer, SIZE_OF_BUFFER);
#ifdef DEBUG
            Serial.print("Received message size: ");
            Serial.println(len);
#endif
            for (int i = 0; i < len / 4; i++)
            {
                float f = ((float *)buffer)[i];
                receivedTrajectory.push_back(f);
            }
        }
        message = Message::parse(receivedTrajectory, senderId);

        // Then send a command
        VecFloat serializedMessage;
        if (Match::getMatchStarted())
        {
            // send a match state message
            MatchStateMessage newMessage = MatchStateMessage(true, Match::getMatchTimeSeconds(), 10);
            serializedMessage = newMessage.serialize();
        }
        else
        {
            bool need_stop_pami = false;
            if (message->get_message_type() == MessageType::PAMI_REPORT)
            {
                if (static_cast<PamiReportMessage *>(message.get())->matchStarted_)
                {
                    need_stop_pami = true;
                }
            }
            if (need_stop_pami)
            {
                // need to stop the pami: send a matchState
                MatchStateMessage newMessage = MatchStateMessage(false, 0.0, 10);
                serializedMessage = newMessage.serialize();
            }
            else
            {
                // send a configuration message
                ConfigurationMessage newMessage = ConfigurationMessage(Match::getSide(), 10);
                serializedMessage = newMessage.serialize();
            }
        }

        for (uint i = 0; i < serializedMessage.size(); i++)
        {
            ((float *)sendBuffer)[i] = serializedMessage[i];
        }

        int sizeToWrite = serializedMessage.size() * 4;

        // If client is still connected, send reply
        if (client.connected())
        {
#ifdef DEBUG
            Serial.print("Connected to: ");
            Serial.println(remoteIP);
#endif
            int sizeOfSentMessage = client.write_P(sendBuffer, sizeToWrite);
#ifdef DEBUG
            Serial.print("Sent message size: ");
            Serial.print(sizeOfSentMessage);
            Serial.print(" expected ");
            Serial.println(sizeToWrite);
#endif

            // Close connection
            client.stop();
        }
#ifdef DEBUG
        else
        {
            Serial.println("Could not reply");
        }
#endif
    }
#else
    sockaddr_in client_addr;
    socklen_t sin_size;
    // accepting connection request
    int clientSocket = accept(serverSocket, (struct sockaddr *)&client_addr, &sin_size);

    if (clientSocket >= 0)
    {
        unsigned char *ip = (unsigned char *)&(client_addr.sin_addr);
        uint8_t senderId = ip[3];
#ifdef DEBUG
        Serial.print("Client socket: ");
        Serial.println(clientSocket);
        Serial.print("Accepted connection from: ");
        Serial.println(inet_ntoa(client_addr.sin_addr));
        Serial.print("Sender ID is: ");
        Serial.println(senderId);
#endif

        // recieving data
        int sizeofreceiveddata;
        receivedTrajectory.clear();

        while ((sizeofreceiveddata = recv(clientSocket, buffer, SIZE_OF_BUFFER * 4, 0)) > 0)
        {
            for (int i = 0; i < sizeofreceiveddata / 4; i++)
            {
                float f = ((float *)buffer)[i];
                receivedTrajectory.push_back(f);
            }
        }

        // end the connection to the client
        close(clientSocket);
#ifdef DEBUG
        // parse the message
        Serial.print("Message length: ");
        Serial.println(receivedTrajectory.size());
        for (auto f : receivedTrajectory)
        {
            Serial.print(f);
            Serial.print(" ");
        }
        Serial.println();
#endif
        message = Message::parse(receivedTrajectory, senderId);
    }
#endif

    return message;
};

void MessageReceiver::stopReceiving()
{
    stopReceiving_ = true;
}