#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>
#include <WiFiHandler.hpp>

#include <Arduino.h>
#include <CRC8.h>

#include <Robot.hpp>
#include <parameters.hpp>


#define MAX_SEND_SIZE 512
#define MAX_RECV_SIZE 512
#define TCP_SERVER_PORT 778

WiFiClient client;
IPAddress serverAddress(MIAM_SCD_ADDRESS);

// Access to data wifi tx is protected using a mutex
// It is accessed in taskSerialMonitor and taskSendDataToServer
SemaphoreHandle_t xSemaphore = NULL;


// message type, id of sender, 3 other floats = 5*4 char
#define MAX_SIZE_OF_PAMI_REPORT 100


#include "esp_log.h"
static const char* TAG = "Robot.cpp";

#define DEBUG_PRINT(x) ESP_LOGV(TAG, "%s", x)
#define DEBUG_PRINTLN(x) ESP_LOGV(TAG, "%s", x)

void task_report_broadcast(void *parameters)
{
    Robot *robot = Robot::getInstance();
    uint8_t *buffer = new uint8_t[MAX_SIZE_OF_PAMI_REPORT / 4];
    int sizeOfMessage = 0;

    client.setTimeout(1);

    for (;;)
    {
        if (!client.connected())
        {
            DEBUG_PRINTLN(">> client not connected");
            if (client.connect(serverAddress, TCP_SERVER_PORT)) 
            {
                DEBUG_PRINTLN("connected to server");
            }
            else
            {
                DEBUG_PRINTLN("could not connect to server");
            }
        }

        // if the robot is in match, stop listening to save processing power
        if (robot->currentRobotState_ == RobotState::MATCH_STARTED_ACTION ||
            robot->currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH)
        {
            taskYIELD();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        FullPamiReportMessage report = robot->get_pami_report();
        sizeOfMessage = report.serialize(buffer, MAX_SIZE_OF_PAMI_REPORT);

        DEBUG_PRINTLN("Sending message");
        for (uint i=0; i<sizeOfMessage; i++)
        {
            DEBUG_PRINT(buffer[i]);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINTLN();

        if(xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            DEBUG_PRINTLN(sizeOfMessage);

            int sizeOfSent = client.write(buffer, sizeOfMessage);
            if (sizeOfSent <= 0) {
                DEBUG_PRINTLN("[ERROR] Send data failed");
            }

            xSemaphoreGive(xSemaphore);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        else
        {
            DEBUG_PRINTLN("taskSendDataToServer: Semaphore not taken");
        }

        int size = 0;
        while (client.available() > 0 && size < MAX_RECV_SIZE)
        {
            buffer[size++] = client.read();
        }
        if (size < 0)
        {
            DEBUG_PRINTLN("[ERROR] Receive data failed");
        }
        else if (size > 0)
        {
            // DEBUG_PRINT("[CLIENT] Receive data from server: ");
            std::shared_ptr<Message> message = Message::parse(buffer, size, 10);
            Robot::getInstance()->notify_new_message(message);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}


namespace MessageHandler
{
    void startReportBroadcast()
    {
        xSemaphore = xSemaphoreCreateMutex();

        xTaskCreate(
            task_report_broadcast,
            "task_report_broadcast",
            30000,
            NULL,
            30,
            NULL);
    }
}