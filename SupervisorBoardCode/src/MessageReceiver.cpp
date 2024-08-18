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
char* buffer; 
char* sendBuffer;

#ifdef USE_ASYNCTCP

SemaphoreHandle_t xSemaphore_new_message = NULL;
std::shared_ptr<Message > new_message;
bool new_message_received = false;

/* clients events */
static void handleError(void* arg, AsyncClient* client, int8_t error)
{
    Serial.println("handleError");
  //(void) arg;

  // Serial.printf("\nConnection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void* arg, AsyncClient* client, void *data, size_t len)
{
    Serial.println("handleData");
  //(void) arg;

  // Serial.printf("\nData received from client %s \n", client->remoteIP().toString().c_str());
  Serial.write((uint8_t*)data, len);

    in_addr adress;
    IPAddress remoteIP = client->remoteIP();
    inet_aton(remoteIP.toString().c_str(), &adress);
    unsigned char *ip = (unsigned char *)&(adress.s_addr);
    uint8_t senderId = ip[3];
    #ifdef DEBUG
    Serial.print(">>>> Client is connected: ");
    Serial.println(client->remoteIP());
#endif

    std::shared_ptr<Message > message = Message::parse((float*) data, len/4, senderId);

    // Then send a command
    int sizeToWrite = 0;
    if (Match::getMatchStarted())
    {
        // send a match state message
        MatchStateMessage newMessage = MatchStateMessage(true, Match::getMatchTimeSeconds(), 10);
        sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
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
            sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
        }
        else
        {
            // send a configuration message
            ConfigurationMessage newMessage = ConfigurationMessage(Match::getSide(), Match::getStopMotors(), 10);
            sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
        }
    }

  // reply to client
  if (client->canSend())
  {
#ifdef DEBUG
    Serial.print("Connected to: ");
    Serial.println(remoteIP);
#endif
    int sizeOfSentMessage = client->add(sendBuffer, sizeToWrite*4);
    client->send();
#ifdef DEBUG
    Serial.print("Sent message size: ");
    Serial.print(sizeOfSentMessage);
    Serial.print(" expected ");
    Serial.println(sizeToWrite*4);
#endif
  }

  // trigger new message
//   if (xSemaphoreTake(xSemaphore_new_message, portMAX_DELAY))
//   {
    new_message_received = true;
    new_message = message;
//     xSemaphoreGive(xSemaphore_new_message);
//   }
    client->close(true);
}

static void handleDisconnect(void* arg, AsyncClient* client)
{
    Serial.println("handleDisconnect");
  //(void) arg;

  // Serial.printf("\nClient %s disconnected\n", client->remoteIP().toString().c_str());
  // delete client;
  delete client;
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time)
{

    Serial.println("handleTimeOut");
//   (void) arg;
//   (void) time;

  // Serial.printf("\nClient ACK timeout ip: %s\n", client->remoteIP().toString().c_str());
}


/* server events */
static void handleNewClient(void* arg, AsyncClient* client)
{
    Serial.println("handleNewClient");
  //(void) arg;

  // Serial.printf("\nNew client has been connected to server, IP: %s", client->remoteIP().toString().c_str());

  // register events
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeOut, NULL);
}
#endif

MessageReceiver::MessageReceiver(){
#ifdef USE_WIFICLIENT_API
    server = new WiFiServer(778);
#else
#ifdef USE_ASYNCTCP
    // xSemaphore_new_message = xSemaphoreCreateMutex();
    server = new AsyncServer(778);
#endif
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
#ifdef USE_ASYNCTCP
    server->onClient(&handleNewClient, NULL);
    Serial.println("Beginning server");
    server->begin();
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
#endif
}

MessageReceiver::~MessageReceiver()
{
#ifndef USE_WIFICLIENT_API
#ifndef USE_ASYNCTCP
    // closing the socket.
    close(serverSocket);
#endif
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

#ifdef USE_WIFICLIENT_API
    WiFiClient client = server->available();
    client.setTimeout(1);
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
        int len = 0;
        // Wait for client sending message
        // timeout = 50ms
        long startTime = millis();
        while (millis() - startTime < 50 && client.connected() && !client.available())
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // Read message
        while (client.connected() && client.available())
        {
            len += client.read((uint8_t *)&(buffer[len]), SIZE_OF_BUFFER-len);
#ifdef DEBUG
            Serial.print("Received message size: ");
            Serial.println(len);
#endif
        }
        message = Message::parse((float*) buffer, len/4, senderId);

        // Then send a command
        int sizeToWrite = 0;
        if (Match::getMatchStarted())
        {
            // send a match state message
            MatchStateMessage newMessage = MatchStateMessage(true, Match::getMatchTimeSeconds(), 10);
            sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
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
                sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
            }
            else
            {
                // send a configuration message
                ConfigurationMessage newMessage = ConfigurationMessage(Match::getSide(), Match::getStopMotors(), 10);
                sizeToWrite = newMessage.serialize((float *) sendBuffer, SIZE_OF_BUFFER/4);
            }
        }

        // If client is still connected, send reply
        if (client.connected())
        {
#ifdef DEBUG
            Serial.print("Connected to: ");
            Serial.println(remoteIP);
#endif
            int sizeOfSentMessage = client.write_P(sendBuffer, sizeToWrite*4);
#ifdef DEBUG
            Serial.print("Sent message size: ");
            Serial.print(sizeOfSentMessage);
            Serial.print(" expected ");
            Serial.println(sizeToWrite*4);
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
#ifdef USE_ASYNCTCP
    // Receiving and replying is handled via async callbacks
    // if (xSemaphoreTake(xSemaphore_new_message, portMAX_DELAY))
    // {    
        if (new_message_received)
        {
            message = new_message;
            new_message_received = false;
        }
    //     xSemaphoreGive(xSemaphore_new_message);
    // } 
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
#endif

    return message;
};

void MessageReceiver::stopReceiving()
{
    stopReceiving_ = true;
}