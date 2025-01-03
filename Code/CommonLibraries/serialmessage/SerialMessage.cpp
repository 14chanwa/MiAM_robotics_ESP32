#include "SerialMessage.hpp"
#include "CRC.h"


SerialMessage::SerialMessage(const uint8_t* message, const uint sizeOfMessage)
{
  if (sizeOfMessage > 2)
  {
    // Check the CRC
    uint8_t challenge_crc = message[sizeOfMessage-1];
    uint8_t actual_crc = calcCRC8(message, sizeOfMessage-1);
    
    if (actual_crc == challenge_crc)
    {
      messageType_ = (SerialMessageType)message[0];
      for (uint i=1; i<sizeOfMessage-1; i++)
      {
        messagePayload_.push_back(message[i]);
      }
    }
  }

  messageType_ = SerialMessageType::ACK_CRC_ERROR;
}


uint SerialMessage::serialize(uint8_t* results, uint maxSize)
{
  // Size out of range
  if (messagePayload_.size() + 2 > maxSize)
  {
    return 0;
  }

  uint sizeOfMessage = 0;
  // Mesage type
  results[sizeOfMessage++] = (uint8_t) messageType_;
  for (uint i=0; i<messagePayload_.size(); i++)
  {
    results[sizeOfMessage++] = messagePayload_.at(i);
  }
  // CRC
  results[sizeOfMessage++] = calcCRC8(results, sizeOfMessage);
  return sizeOfMessage;
}