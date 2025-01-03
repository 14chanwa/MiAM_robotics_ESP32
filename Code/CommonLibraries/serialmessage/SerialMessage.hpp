#include <Arduino.h>
#include <vector>

enum SerialMessageType
{
  ACK_OK = 0,
  ACK_NOK = 1,
  ACK_CRC_ERROR = 2,
  TRANSMIT = 3
};

class SerialMessage
{
public:
  SerialMessage(SerialMessageType mt) : messageType_(mt) {};
  SerialMessage(SerialMessageType mt, std::vector<uint8_t> pl) : messageType_(mt), messagePayload_(pl) {};
  SerialMessage(const uint8_t* message, const uint sizeOfMessage);

  uint serialize(uint8_t* results, uint maxSize);

  SerialMessageType messageType_;
  std::vector<uint8_t > messagePayload_;
};
