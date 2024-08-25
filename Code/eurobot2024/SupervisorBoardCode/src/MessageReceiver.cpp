#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>

#include <SampledTrajectory.h>
#include <Match.hpp>

#include <Arduino.h>
#include <PAMIStates.hpp>

#define DEBUG_LOG

#ifdef DEBUG_LOG
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define LISTENING_PORT 778
#define SIZE_OF_BUFFER 512

namespace MessageReceiver {

    std::vector<float > receivedTrajectory;
    bool stopReceiving_ = false;
    char* buffer; 
    char* sendBuffer;

    SemaphoreHandle_t xSemaphore_new_message = NULL;

    AsyncServer* server;
    
    /* clients events */
    static void handleError(void* arg, AsyncClient* client, int8_t error)
    {
        DEBUG_PRINTLN("handleError");
    }

    static void handleData(void* arg, AsyncClient* client, void *data, size_t len)
    {
        DEBUG_PRINTLN("handleData");
        in_addr adress;
        IPAddress remoteIP = client->remoteIP();
        inet_aton(remoteIP.toString().c_str(), &adress);
        unsigned char *ip = (unsigned char *)&(adress.s_addr);
        uint8_t senderId = ip[3];
        DEBUG_PRINT(">>>> Client is connected: ");
        DEBUG_PRINTLN(client->remoteIP());

        std::shared_ptr<Message > message = Message::parse((float*) data, len/4, senderId);

        // Semaphore is used since shared buffer is used
        if (xSemaphoreTake(xSemaphore_new_message, portMAX_DELAY))
        {
            // Register new message to PAMIs
            DEBUG_PRINTLN("TCP received message");
            PAMIStates::registerMessage(message);
            
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
                DEBUG_PRINT("Connected to: ");
                DEBUG_PRINTLN(remoteIP);
                int sizeOfSentMessage = client->add(sendBuffer, sizeToWrite*4);
                if (client->send())
                {
                    DEBUG_PRINT("Sent message size: ");
                    DEBUG_PRINT(sizeOfSentMessage);
                    DEBUG_PRINT(" expected ");
                    DEBUG_PRINTLN(sizeToWrite*4);
                }
                else
                {
                    DEBUG_PRINTLN("Could not send message");
                }
                
            }

            // Release semaphore
            xSemaphoreGive(xSemaphore_new_message);
        }
    }

    static void handleDisconnect(void* arg, AsyncClient* client)
    {
        DEBUG_PRINTLN("handleDisconnect");
        delete client;
    }

    static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time)
    {
        DEBUG_PRINTLN("handleTimeOut");
        client->close();
    }

    /* server events */
    static void handleNewClient(void* arg, AsyncClient* client)
    {
        DEBUG_PRINTLN("handleNewClient");
        // register events
        client->setRxTimeout(3);
        client->onData(&handleData, NULL);
        client->onError(&handleError, NULL);
        client->onDisconnect(&handleDisconnect, NULL);
        client->onTimeout(&handleTimeOut, NULL);
    }

    void startListening()
    {
        xSemaphore_new_message = xSemaphoreCreateMutex();
        sendBuffer = new char[SIZE_OF_BUFFER]();

        server = new AsyncServer(LISTENING_PORT);
        server->onClient(&handleNewClient, NULL);
        DEBUG_PRINTLN("Beginning server");
        server->begin();
    }
};
