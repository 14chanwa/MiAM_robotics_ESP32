#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>
// #include <WiFiHandler.hpp>
#include <TFTScreen.hpp>
#include <PAMIStates.hpp>

#include <Arduino.h>

// #include <Robot.hpp>
// #include <parameters.hpp>

// message type, id of sender, 3 other floats = 5*4 char
#define MAX_SIZE_OF_PAMI_REPORT 50

MessageReceiver messageReceiver;
// MessageReceiverUDP messageReceiverUDP;

void task_messageReceiver(void* parameters)
{
    // Robot* robot = Robot::getInstance();
    for (;;)
    {
        // Serial.println(">> TCP receiver standby...");
        std::shared_ptr<Message > message = messageReceiver.receive();
        if (message != nullptr)
        {
            Serial.println("TCP received message");
            // robot->notify_new_message(message);
            PAMIStates::registerMessage(message);
        }
    }
}

// void task_messageReceiverUDP(void* parameters)
// {
//     // Robot* robot = Robot::getInstance();
//     for (;;)
//     {
//         // Serial.println(">> UDP receiver standby...");
//         std::shared_ptr<Message > message = messageReceiverUDP.receive();
//         if (message != nullptr)
//         {
//             // Serial.println("UDP received message");
//             // robot->notify_new_message(message);
//             PAMIStates::registerMessage(message);
//         }
//     }
// }


// void task_report_broadcast(void* parameters)
// {
//     // Robot* robot = Robot::getInstance();
//     char* buffer = new char[MAX_SIZE_OF_PAMI_REPORT];
//     uint sizeOfMessage;

//     for(;;)
//     {
//         PamiReportMessage report = robot->get_pami_report();
//         VecFloat serialized_report = report.serialize();
//         if (serialized_report.size() * 4 <= MAX_SIZE_OF_PAMI_REPORT)
//         {
//             for (uint i=0; i<serialized_report.size(); i++)
//             {
//                 buffer[i*4] = serialized_report.at(i);
//             }
//             sizeOfMessage = serialized_report.size() * 4;
    
//             bool success = WiFiHandler::sendTCPMessage(
//                 buffer,
//                 sizeOfMessage,
//                 MIAM_SCD_ADDRESS,
//                 MIAM_SCD_PORT
//             );
//             if (!success)
//             {
//                 Serial.println("Failed to send report to SCD");
//             }
//             else
//             {
//                 Serial.println("Sent report SCD");
//             }
//         }
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }


namespace MessageHandler{

    void startListening()
    {
        messageReceiver.begin();
        // messageReceiverUDP.begin();

        xTaskCreatePinnedToCore(
            task_messageReceiver,
            "task_messageReceiver",
            100000,
            NULL,
            40,
            NULL,
            1
        );

        // xTaskCreatePinnedToCore(
        //     task_messageReceiverUDP,
        //     "task_messageReceiverUDP",
        //     50000,
        //     NULL,
        //     40,
        //     NULL,
        //     1
        // );
    }

    void startReportBroadcast()
    {
        // xTaskCreate(
        //     task_report_broadcast,
        //     "task_report_broadcast",
        //     20000,
        //     NULL,
        //     30,
        //     NULL
        // );
    }
}