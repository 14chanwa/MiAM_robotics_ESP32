#include <Arduino.h>
#include <MessageReceiver.hpp>
#include <cstring>

#include <WiFi.h>
#include "mqtt_client.h"

#include <SampledTrajectory.h>
#include <Match.hpp>

#include <Arduino.h>
#include <PAMIStates.hpp>

#include <secret.hpp>

#define DEBUG_LOG

#ifdef DEBUG_LOG
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// #define LISTENING_PORT 778
#define SIZE_OF_BUFFER 512

// MQTT configuration
const char MQTT_BROKER_ADRRESS[] = MQTT_SERVER_IP;
const int MQTT_PORT = 1883;
const char MQTT_CLIENT_ID[] = "esp32-SCD";
const char MQTT_USERNAME[] = "";
const char MQTT_PASSWORD[] = "";

// The MQTT topics that ESP32 should publish/subscribe
const char PUBLISH_TOPIC[] = "match-state";
const char SUBSCRIBE_TOPIC[] = "pami-state";

const int PUBLISH_INTERVAL = 1;  // 1 seconds

WiFiClient espClient;
// PubSubClient client(espClient);
static const char *TAG = "mqtt_example";

esp_mqtt_client_config_t mqtt_cfg;
esp_mqtt_client_handle_t mqtt_client;

namespace MessageReceiver {

    std::vector<float > receivedTrajectory;
    bool stopReceiving_ = false;
    uint8_t* sendBuffer;

    SemaphoreHandle_t xSemaphore_new_message = NULL;

    // AsyncServer* server;
    
    /* clients events */
    // static void handleError(void* arg, AsyncClient* client, int8_t error)
    // {
    //     DEBUG_PRINTLN("handleError");
    // }

    // static void handleData(void* arg, AsyncClient* client, void *data, size_t len)
    // void callback(char* topic, byte* data, unsigned int len)

    //MQTT event handler function
    static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
    {
        esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
        esp_mqtt_client_handle_t client = event->client;
        int msg_id;
        switch ((esp_mqtt_event_id_t)event_id) {
            case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            //Subscribe to topic /topic/test
            msg_id = esp_mqtt_client_subscribe(client, SUBSCRIBE_TOPIC, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
            case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
            case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
            case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s\r\n", event->data_len, event->data);
            Serial.print("event->data_len: ");
            Serial.println(event->data_len);
            Serial.print("event->data: ");
            for (uint i=0; i<event->data_len; i++)
            {
                Serial.print((uint8_t)(event->data[i]));
                Serial.print(" ");
            }
            Serial.println();
            if (xSemaphoreTake(xSemaphore_new_message, portMAX_DELAY))
            {
                std::shared_ptr<Message > message = Message::parse((uint8_t*)event->data, event->data_len);
                PAMIStates::registerMessage(message);
                // Release semaphore
                xSemaphoreGive(xSemaphore_new_message);
            }
            break;
            case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
            default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
        }
    }

    // void callback(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    // {
    //     DEBUG_PRINTLN("handleData");
    //     // in_addr adress;
    //     // IPAddress remoteIP = client->remoteIP();
    //     // inet_aton(remoteIP.toString().c_str(), &adress);
    //     // unsigned char *ip = (unsigned char *)&(adress.s_addr);
    //     // uint8_t senderId = ip[3];
    //     // DEBUG_PRINT(">>>> Client is connected: ");
    //     // DEBUG_PRINTLN(client->remoteIP());

    //     // DEBUG_PRINTLN(topic);

    //     std::shared_ptr<Message > message = Message::parse((uint8_t*)data, len);
    //     // std::shared_ptr<Message > message = Message::parse((float*) data, len/4, senderId);

    //     // Semaphore is used since shared buffer is used
    //     if (xSemaphoreTake(xSemaphore_new_message, portMAX_DELAY))
    //     {
    //         // Register new message to PAMIs
    //         DEBUG_PRINTLN("received message");
    //         PAMIStates::registerMessage(message);
            
    //         // // Then send a command
    //         // int sizeToWrite = 0;
    //         // if (Match::getMatchStarted())
    //         // {
    //         //     // send a match state message
    //         //     MatchStateMessage newMessage = MatchStateMessage(true, Match::getMatchTimeSeconds(), 10);
    //         //     sizeToWrite = newMessage.serialize(sendBuffer, SIZE_OF_BUFFER);
    //         // }
    //         // else
    //         // {
    //         //     bool need_stop_pami = false;
    //         //     if (message->get_message_type() == MessageType::PAMI_REPORT)
    //         //     {
    //         //         if (static_cast<PamiReportMessage *>(message.get())->matchStarted_)
    //         //         {
    //         //             need_stop_pami = true;
    //         //         }
    //         //     }
    //         //     if (need_stop_pami)
    //         //     {
    //         //         // need to stop the pami: send a matchState
    //         //         MatchStateMessage newMessage = MatchStateMessage(false, 0.0, 10);
    //         //         sizeToWrite = newMessage.serialize(sendBuffer, SIZE_OF_BUFFER);
    //         //     }
    //         //     else
    //         //     {
    //         //         // send a configuration message
    //         //         ConfigurationMessage newMessage = ConfigurationMessage(Match::getSide(), Match::getStopMotors(), 10);
    //         //         sizeToWrite = newMessage.serialize(sendBuffer, SIZE_OF_BUFFER);
    //         //     }
    //         // }

    //         // // reply to client
    //         // if (client->canSend())
    //         // {
    //         //     DEBUG_PRINT("Connected to: ");
    //         //     DEBUG_PRINTLN(remoteIP);
    //         //     int sizeOfSentMessage = client->add((char*)sendBuffer, sizeToWrite);
    //         //     if (client->send())
    //         //     {
    //         //         DEBUG_PRINT("Sent message size: ");
    //         //         DEBUG_PRINT(sizeOfSentMessage);
    //         //         DEBUG_PRINT(" expected ");
    //         //         DEBUG_PRINTLN(sizeToWrite);
    //         //     }
    //         //     else
    //         //     {
    //         //         DEBUG_PRINTLN("Could not send message");
    //         //     }
                
    //         // }

    //         // Release semaphore
    //         xSemaphoreGive(xSemaphore_new_message);
    //     }
    // }

    // static void handleDisconnect(void* arg, AsyncClient* client)
    // {
    //     DEBUG_PRINTLN("handleDisconnect");
    //     delete client;
    // }

    // static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time)
    // {
    //     DEBUG_PRINTLN("handleTimeOut");
    //     client->close();
    // }

    // /* server events */
    // static void handleNewClient(void* arg, AsyncClient* client)
    // {
    //     DEBUG_PRINTLN("handleNewClient");
    //     // register events
    //     client->setRxTimeout(3);
    //     client->onData(&handleData, NULL);
    //     client->onError(&handleError, NULL);
    //     client->onDisconnect(&handleDisconnect, NULL);
    //     client->onTimeout(&handleTimeOut, NULL);
    // }

    void reconnect() {
        // // Loop until we're reconnected
        // while (!mqtt_client.connected()) {
        //     Serial.print("Attempting MQTT connection...");
        //     // Attempt to connect
        //     if (client.connect(MQTT_CLIENT_ID)) {
        //     Serial.println("connected");
        //     // Subscribe
        //     client.subscribe(SUBSCRIBE_TOPIC);
        //     } else {
        //     Serial.print("failed, rc=");
        //     Serial.print(client.state());
        //     Serial.println(" try again in 5 seconds");
        //     // Wait 5 seconds before retrying
        //     delay(5000);
        //     }
        // }
    }

    void task_broadcast_match_state(void* parameters)
    {
        for(;;)
        {
            //reconnect();
            // send a match state message
            FullMatchStateMessage newMessage = FullMatchStateMessage(Match::getSide(), Match::getMatchStarted(), Match::getMatchTimeSeconds(), 10);
            int sizeToWrite = newMessage.serialize(sendBuffer, SIZE_OF_BUFFER);
            //client.publish(PUBLISH_TOPIC, sendBuffer, sizeToWrite);
            Serial.println("Sending message");
            for (uint i=0; i<sizeToWrite; i++)
            {
                Serial.print(sendBuffer[i]);
                Serial.print(" ");
            }
            Serial.println();
            esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (const char *)sendBuffer, sizeToWrite, 0, 0);
            delay(500);
        }
    }

    void startListening()
    {
        xSemaphore_new_message = xSemaphoreCreateMutex();
        sendBuffer = new uint8_t[SIZE_OF_BUFFER]();

        // server = new AsyncServer(LISTENING_PORT);
        // server->onClient(&handleNewClient, NULL);
        DEBUG_PRINTLN("Beginning server");
        // server->begin();

        // client.setServer(MQTT_BROKER_ADRRESS, MQTT_PORT);
        // client.setCallback(callback);

        esp_log_level_set("*", ESP_LOG_INFO);
        esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
        esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
        esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
        esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
        esp_log_level_set("transport", ESP_LOG_VERBOSE);
        esp_log_level_set("outbox", ESP_LOG_VERBOSE);

        char buffer[50];
        sprintf(buffer, "mqtt://%s/", MQTT_BROKER_ADRRESS);

        mqtt_cfg.uri = buffer;
        mqtt_cfg.port = MQTT_PORT;
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(mqtt_client);

        xTaskCreatePinnedToCore(
            task_broadcast_match_state,
            "task_broadcast_match_state",
            20000,
            NULL,
            50,
            NULL,
            0
        );
    }
};
