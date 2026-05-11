
#include <PAMIStates.hpp>
#include <Arduino.h>

#define DEBUG_PAMISTATES

#define PAMI_TIMEOUT 2000
#define MAX_PAMI_ID 6

long lastMillisRegisterMessage[MAX_PAMI_ID] = {0, 0, 0, 0, 0, 0};
FullPamiReportMessage default_message = FullPamiReportMessage(false, 0.0, PlayingSide::BLUE_SIDE, 0.0, RobotPosition(0, 0, 0), 255);
std::vector<FullPamiReportMessage > pamiReportMessage({default_message, default_message, default_message, default_message, default_message, default_message});

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
    Serial.print("  ");
    Serial.println(MessageType::FULL_PAMI_REPORT);
#endif
    if (message->get_message_type() == MessageType::FULL_PAMI_REPORT && senderID >= 1 && senderID <= MAX_PAMI_ID)
    {
#ifdef DEBUG_PAMISTATES
        Serial.println("Registering message");
#endif
        FullPamiReportMessage newMessage = *static_cast<FullPamiReportMessage* >(message.get());
        pamiReportMessage[senderID-1] = newMessage;
        lastMillisRegisterMessage[senderID-1] = millis();
    }
}

FullPamiReportMessage readPAMIMessage(uint8_t pamiID)
{
    if (pamiID >= 1 && pamiID <= MAX_PAMI_ID)
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
    for (uint i = 1; i<=MAX_PAMI_ID; i++)
        maxValue = std::max(maxValue, lastMillisRegisterMessage[i]);
    return maxValue;
}

}

