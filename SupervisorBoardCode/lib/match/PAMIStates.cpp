
#include <PAMIStates.hpp>
#include <Arduino.h>

// #define DEBUG_PAMISTATES

#define PAMI_TIMEOUT 2000

long lastMillisRegisterMessage[5] = {0, 0, 0, 0, 0};
PamiReportMessage default_message = PamiReportMessage(false, 0, PlayingSide::BLUE_SIDE, 255);
std::vector<PamiReportMessage > pamiReportMessage({default_message, default_message, default_message, default_message, default_message});

namespace PAMIStates
{

void registerMessage(std::shared_ptr<Message > message)
{
    uint8_t senderID = message->get_sender_id();
#ifdef DEBUG_PAMISTATES
    // lastMillisRegisterMessage = millis();
    Serial.print("Received message from: ");
    Serial.println(message->get_sender_id());
    Serial.print("Message type is ");
    Serial.print(message->get_message_type());
    Serial.print(" expected ");
    Serial.println(MessageType::PAMI_REPORT);
#endif
    if (message->get_message_type() == MessageType::PAMI_REPORT && senderID-10 >= 1 && senderID-10 <= 5)
    {
#ifdef DEBUG_PAMISTATES
        Serial.println("Registering message");
#endif
        PamiReportMessage newMessage = *static_cast<PamiReportMessage* >(message.get());
        pamiReportMessage[senderID-10-1] = newMessage;
        lastMillisRegisterMessage[senderID-10-1] = millis();
    }
}

PamiReportMessage readPAMIMessage(uint8_t pamiID)
{
    if (pamiID >= 1 && pamiID <= 5)
    {
        if (millis() - lastMillisRegisterMessage[pamiID-1] <= PAMI_TIMEOUT)
        {
            return pamiReportMessage[pamiID-1];
        }
    }
    return default_message;
}

long readLastMessageTime()
{
    long maxValue = lastMillisRegisterMessage[0];
    for (char i = 1; i<5; i++)
        maxValue = std::max(maxValue, lastMillisRegisterMessage[i]);
    return maxValue;
}

}

