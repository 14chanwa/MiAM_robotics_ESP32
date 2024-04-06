#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>
#include <WiFiHandler.hpp>

#include <Arduino.h>

#include <Robot.hpp>
#include <parameters.hpp>

// message type, id of sender, 3 other floats = 5*4 char
#define MAX_SIZE_OF_PAMI_REPORT 50

MessageReceiver messageReceiver;
MessageReceiverUDP messageReceiverUDP;

void task_messageReceiver(void* parameters)
{
    Robot* robot = Robot::getInstance();
    for (;;)
    {
        Serial.println(">> TCP receiver standby...");
        std::shared_ptr<Message > message = messageReceiver.receive();
        Serial.println("TCP received message");
        robot->notify_new_message(message);
    }
}

void task_messageReceiverUDP(void* parameters)
{
    Robot* robot = Robot::getInstance();
    for (;;)
    {
        Serial.println(">> UDP receiver standby...");
        std::shared_ptr<Message > message = messageReceiverUDP.receive();
        Serial.println("UDP received message");
        robot->notify_new_message(message);
    }
}


void task_report_broadcast(void* parameters)
{
    Robot* robot = Robot::getInstance();
    float* buffer = new float[MAX_SIZE_OF_PAMI_REPORT/4];
    uint sizeOfMessage;

    for(;;)
    {
        PamiReportMessage report = robot->get_pami_report();
        VecFloat serialized_report = report.serialize();
        if (serialized_report.size() * 4 <= MAX_SIZE_OF_PAMI_REPORT)
        {
            for (uint i=0; i<serialized_report.size(); i++)
            {
                buffer[i] = serialized_report.at(i);
            }
            sizeOfMessage = serialized_report.size() * 4;
    
            bool success = WiFiHandler::sendTCPMessage(
                (char*)buffer,
                sizeOfMessage,
                MIAM_SCD_ADDRESS,
                MIAM_SCD_PORT
            );
            if (!success)
            {
                Serial.println("Failed to send report to SCD");
            }
            else
            {
                Serial.println("Sent report SCD");
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


namespace MessageHandler{

    void startListening()
    {
        messageReceiver.begin();
        messageReceiverUDP.begin();

        xTaskCreate(
            task_messageReceiver,
            "task_messageReceiver",
            50000,
            NULL,
            40,
            NULL
        );

        xTaskCreate(
            task_messageReceiverUDP,
            "task_messageReceiverUDP",
            50000,
            NULL,
            40,
            NULL
        );
    }

    void startReportBroadcast()
    {
        xTaskCreate(
            task_report_broadcast,
            "task_report_broadcast",
            20000,
            NULL,
            30,
            NULL
        );
    }
}