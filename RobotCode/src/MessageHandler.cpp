#include <MessageHandler.hpp>
#include <MessageReceiver.hpp>

#include <Arduino.h>

#include <Robot.hpp>


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


namespace MessageHandler{

    void start()
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
}