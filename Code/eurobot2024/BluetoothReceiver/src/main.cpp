#include <Arduino.h>
#include <Ps3Controller.h>

#include <HardwareSerial.h>
#include <PacketSerial.h>
#include <FastCRC.h>

#include <secret.hpp>

#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 1024
uint8_t* tx_buffer;
#define PACKETSERIAL_SEND_PERIOD_MS 50 // only send packet serial if last message was x ms before

#define RX_PIN 16
#define TX_PIN 17
#define BAUDRATE 115200


PacketSerial packetSerial;
FastCRC16 fastCRC; 

bool ledState = false;
#define LED_PWM_CHANNEL 0
#define LED_PWM_FREQUENCY 1000
#define LED_PWM_RESOLUTION_BIT 8
#define LED_PWM_DUTY_OFF 0
#define LED_PWM_DUTY_ON 32

long lastMillis = 0;

void notify()
{
  log_i("Received new data");

  // Only send packet if last one was old enough
  if (millis() - lastMillis > PACKETSERIAL_SEND_PERIOD_MS)
  {
    ledState = !ledState;
    ledcWrite(LED_PWM_CHANNEL, ledState ? LED_PWM_DUTY_ON : LED_PWM_DUTY_OFF);

    memcpy(tx_buffer, &Ps3.data, sizeof(Ps3.data));
    uint16_t crc = fastCRC.mcrf4xx(tx_buffer, sizeof(Ps3.data));
    memcpy(&(tx_buffer[sizeof(Ps3.data)]), &crc, sizeof(crc));

    packetSerial.send(tx_buffer, sizeof(Ps3.data) + sizeof(crc));
    log_i("Sent packetSerial length %d", sizeof(Ps3.data) + sizeof(crc));

    lastMillis = millis();
  }
}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  Serial.print("Received: ");
  Serial.write(buffer, size);
  Serial.println();
}

void setup()
{
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  Serial.begin(115200);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  ledcAttachPin(LED_BUILTIN, LED_PWM_CHANNEL);
  ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION_BIT);
  ledcWrite(LED_PWM_CHANNEL, LED_PWM_DUTY_OFF);

  tx_buffer = new uint8_t[TX_BUFFER_SIZE];
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  packetSerial.setStream(&Serial1);
  packetSerial.setPacketHandler(&onPacketReceived);

  ledcWrite(LED_PWM_CHANNEL, LED_PWM_DUTY_ON);
  
  // BT PS3 controller
  Ps3.begin(SECRET_BT_MAC_ADDRESS);
  Ps3.attach(notify);
}

void loop()
{
  packetSerial.update();
  vTaskDelay(20 / portTICK_PERIOD_MS);
}
