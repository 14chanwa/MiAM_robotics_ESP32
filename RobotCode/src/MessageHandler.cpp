#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>
#include <WiFiHandler.hpp>

#include <Arduino.h>

#include <Robot.hpp>
#include <parameters.hpp>

// message type, id of sender, 3 other floats = 5*4 char
#define MAX_SIZE_OF_PAMI_REPORT 50

WiFiClient wifiClient;

// MessageReceiver messageReceiver;
// MessageReceiverUDP messageReceiverUDP;

// void task_messageReceiver(void* parameters)
// {
//     Robot* robot = Robot::getInstance();
//     for (;;)
//     {
//         Serial.println(">> TCP receiver standby...");
//         std::shared_ptr<Message > message = messageReceiver.receive();
//         Serial.println("TCP received message");
//         robot->notify_new_message(message);
//     }
// }

// void task_messageReceiverUDP(void* parameters)
// {
//     Robot* robot = Robot::getInstance();
//     for (;;)
//     {
//         Serial.println(">> UDP receiver standby...");
//         std::shared_ptr<Message > message = messageReceiverUDP.receive();
//         Serial.println("UDP received message");
//         robot->notify_new_message(message);
//     }
// }

void task_report_broadcast(void *parameters)
{
    Robot *robot = Robot::getInstance();
    float *buffer = new float[MAX_SIZE_OF_PAMI_REPORT / 4];
    int sizeOfMessage = 0;

    wifiClient.setTimeout(1);

    for (;;)
    {
        // if the robot is in match, stop listening to save processing power
        if (robot->currentRobotState_ == RobotState::MATCH_STARTED_ACTION ||
            robot->currentRobotState_ == RobotState::MATCH_STARTED_FINAL_APPROACH)
        {
            taskYIELD();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        PamiReportMessage report = robot->get_pami_report();
        sizeOfMessage = report.serialize(buffer, MAX_SIZE_OF_PAMI_REPORT / 4);


        bool success = false;

        // Connect to client
        if (wifiClient.connect(MIAM_SCD_ADDRESS, MIAM_SCD_PORT))
        {
            size_t sizeOfSentMessage = wifiClient.write_P((char *)buffer, sizeOfMessage * 4);

            if (sizeOfSentMessage == sizeOfMessage*4)
                success = true;

            if (!success)
                Serial.println("Failed to send report to SCD");
            else
                Serial.println("Sent report SCD");

            // Wait for reply

            while (wifiClient.connected() && !wifiClient.available())
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            
            int len = 0;
            if (wifiClient.available())
            {
                len = wifiClient.read((uint8_t *)buffer, MAX_SIZE_OF_PAMI_REPORT);

                Serial.print("Received message size: ");
                Serial.println(len);
            }

            wifiClient.stop();

            std::shared_ptr<Message> message = Message::parse(buffer, len/4, 10);
            robot->notify_new_message(message);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

namespace MessageHandler
{

    // void startListening()
    // {
    //     messageReceiver.begin();
    //     messageReceiverUDP.begin();

    //     xTaskCreate(
    //         task_messageReceiver,
    //         "task_messageReceiver",
    //         50000,
    //         NULL,
    //         40,
    //         NULL
    //     );
    // }

    void startReportBroadcast()
    {
        xTaskCreate(
            task_report_broadcast,
            "task_report_broadcast",
            20000,
            NULL,
            30,
            NULL);
    }
}