#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>
// #include <WiFiHandler.hpp>

#include <Arduino.h>

#include <Robot.hpp>
#include <parameters.hpp>

#include <PacketSerial.h>
PacketSerial myPacketSerial;
HardwareSerial mySerial(1);

#include <CRC.h>

#define TXD1 17
#define RXD1 16

#include <SerialMessage.hpp>

// message type, id of sender, 3 other floats = 5*4 char
#define MAX_SIZE_OF_PAMI_REPORT 50

// WiFiClient wifiClient;

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
    uint8_t *buffer = new uint8_t[MAX_SIZE_OF_PAMI_REPORT / 4];
    int sizeOfMessage = 0;

    // wifiClient.setTimeout(1);

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
        sizeOfMessage = report.serialize(buffer, MAX_SIZE_OF_PAMI_REPORT);

        // Send over serial
        // TODO add crc


        // std::vector<uint8_t > newPayload;
        // for (uint i=0; i<sizeOfMessage; i++)
        // {
        //     newPayload.push_back(buffer[i]);
        // }

        // SerialMessage newMessage(SerialMessageType::TRANSMIT, newPayload);
        // int sizeOfSent = newMessage.serialize((uint8_t*)buffer, MAX_SIZE_OF_PAMI_REPORT / 4);
        
        // Transfer data over serial
        Serial.print("Sending ");
        Serial.print(sizeOfMessage);
        Serial.println(" over serial");
        myPacketSerial.send((uint8_t*)buffer, sizeOfMessage);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}



// This is our handler callback function.
// When an encoded packet is received and decoded, it will be delivered here.
// The `buffer` is a pointer to the decoded byte array. `size` is the number of
// bytes in the `buffer`.
void onPacketReceived(const uint8_t* buffer, size_t size)
{
    Serial.print("Received message size: ");
    Serial.println(size);

    // // Check CRC
    // if (size > 2)
    // {
    //     // Check the CRC
    //     uint8_t challenge_crc = buffer[size-1];
    //     uint8_t actual_crc = calcCRC8(buffer, size-1);
        
    //     if (actual_crc == challenge_crc)
    //     {
    //         // First byte = message type ; last byte = crc
    //         std::shared_ptr<Message> message = Message::parse((const float*)buffer[1], (size-2)/4, 10);
    //         Robot::getInstance()->notify_new_message(message);
    //     }
    //     else
    //     {
    //         Serial.println("CRC error");
    //     }
    // }

    std::shared_ptr<Message> message = Message::parse(buffer, size, 10);
    Robot::getInstance()->notify_new_message(message);


}

void task_receive_message(void *parameters)
{
    for (;;)
    {
        myPacketSerial.update();
        vTaskDelay(15 / portTICK_PERIOD_MS);;
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
        // Initialize packetserial
        mySerial.begin(115200, SERIAL_8N1, RXD1, TXD1);  // UART setup
        myPacketSerial.setStream(&mySerial);
        myPacketSerial.setPacketHandler(&onPacketReceived);

        xTaskCreate(
            task_report_broadcast,
            "task_report_broadcast",
            20000,
            NULL,
            30,
            NULL);

        xTaskCreate(
            task_receive_message,
            "task_receive_message",
            20000,
            NULL,
            30,
            NULL);
    }
}