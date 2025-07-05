// #include <BluetoothReceiverHandler.hpp>
// #include <PacketSerial.h>
// #include <FastCRC.h>


// namespace bluetooth_receiver_handler
// {
//     #define RX_BUFFER_SIZE 1024

//     #define RX_PIN 16
//     #define TX_PIN 17
//     #define BAUDRATE 115200

//     PacketSerial packetSerial;
//     FastCRC16 fastCRC; 

//     ps3_data_type::ps3_t currentData;
//     ps3_data_type::ps3_t retreivedData;
//     bool newMessageReceived_ = false;

//     SemaphoreHandle_t xMutex = NULL;

//     bool ledState = false;
//     #define LED_PWM_CHANNEL 0
//     #define LED_PWM_FREQUENCY 1000
//     #define LED_PWM_RESOLUTION_BIT 8
//     #define LED_PWM_DUTY_OFF 0
//     #define LED_PWM_DUTY_ON 32

//     void onPacketReceived(const uint8_t* buffer, size_t size)
//     {
//         if (size != sizeof(currentData) + sizeof(uint16_t))
//         {
//             log_e("packetSerial wrong size: %d (expected %d)", size, sizeof(currentData) + sizeof(uint16_t));
//             return;
//         }

//         // Check CRC
//         uint16_t crc = fastCRC.mcrf4xx(buffer, sizeof(ps3_data_type::ps3_t));
//         uint16_t against;
//         memcpy(&against, &(buffer[size - sizeof(crc)]), sizeof(crc));
//         if (against != crc)
//         {
//             log_e("packetSerial failed CRC");
//             return;
//         }
        
//         if (xSemaphoreTake(xMutex, portMAX_DELAY)) 
//         {
//             memcpy(&currentData, buffer, sizeof(ps3_data_type::ps3_t));
//             newMessageReceived_ = true;
//             ledState = !ledState;
//             ledcWrite(LED_PWM_CHANNEL, ledState ? LED_PWM_DUTY_ON : LED_PWM_DUTY_OFF);
//             // log_i("packetSerial received new data");
//             xSemaphoreGive(xMutex);
//         }
//     }

//     void task_receive_serial(void* parameters)
//     {
//         for (;;)
//         {
//             packetSerial.update();
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }
//     }

//     void init()
//     {
//         ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION_BIT);
//         ledcWrite(LED_PWM_CHANNEL, LED_PWM_DUTY_OFF);

//         xMutex = xSemaphoreCreateMutex();
//         Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
//         packetSerial.setStream(&Serial1);
//         packetSerial.setPacketHandler(&onPacketReceived);
//         xTaskCreate(
//             task_receive_serial,
//             "task_receive_serial",
//             10000,
//             NULL,
//             40,
//             NULL
//         );
//         log_i("packetSerial init");
//     }

//     bool newMessageReceived()
//     {
//         return newMessageReceived_;
//     }

//     const ps3_data_type::ps3_t* getData()
//     {
//         if (xSemaphoreTake(xMutex, portMAX_DELAY)) 
//         {
//             memcpy(&retreivedData, &currentData, sizeof(ps3_data_type::ps3_t));
//             newMessageReceived_ = false;
//             // log_i("read new data");
//             xSemaphoreGive(xMutex);
//         }
//         return &retreivedData;
//     }
// }