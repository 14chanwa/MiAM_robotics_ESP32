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
    uint sizeOfMessage;
    VecFloat receivedTrajectory;

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
        VecFloat serialized_report = report.serialize();

        for (uint i = 0; i < serialized_report.size(); i++)
        {
            buffer[i] = serialized_report.at(i);
        }
        sizeOfMessage = serialized_report.size() * 4;

        bool success = false;

        // Connect to client
        if (wifiClient.connect(MIAM_SCD_ADDRESS, MIAM_SCD_PORT))
        {
            size_t sizeOfSentMessage = wifiClient.write_P((char *)buffer, sizeOfMessage);

            if (sizeOfSentMessage == sizeOfMessage)
                success = true;

            if (!success)
                Serial.println("Failed to send report to SCD");
            else
                Serial.println("Sent report SCD");

            // Wait for reply
            receivedTrajectory.clear();

            while (wifiClient.connected() && !wifiClient.available())
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            if (wifiClient.available())
            {
                int len = wifiClient.read((uint8_t *)buffer, MAX_SIZE_OF_PAMI_REPORT);

                Serial.print("Received message size: ");
                Serial.println(len);

                for (int i = 0; i < len / 4; i++)
                {
                    float f = ((float *)buffer)[i];
                    receivedTrajectory.push_back(f);
                }
            }

            wifiClient.stop();

            std::shared_ptr<Message> message = Message::parse(receivedTrajectory, 10);
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