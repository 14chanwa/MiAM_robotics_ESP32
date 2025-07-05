// #include <MessageHandler.hpp>
// #include <Arduino.h>
// #include <parameters.hpp>
// #include <WiFiClient.h>

// #define WRITE_BUFFER_SIZE 100
// #define READ_BUFFER_SIZE 300

// #define RESPONSE_TIMEOUT 150

// namespace message_handler
// {
//     // AsyncClient* client;
//     WiFiClient wifiClient;
//     uint8_t *write_buffer = new uint8_t[WRITE_BUFFER_SIZE];
//     uint8_t *read_buffer = new uint8_t[READ_BUFFER_SIZE];

//     void task_report_broadcast(void *parameters)
//     {
//         wifiClient.setTimeout(3);
//         for (;;)
//         {
//             while (!wifiClient.connected())
//             {
//                 if (!wifiClient.connect(MIAM_SCD_ADDRESS, MIAM_SCD_PORT))
//                 {
//                     log_e("Failed to connect to server");
//                     vTaskDelay(2000 / portTICK_PERIOD_MS);
//                 }
//                 else
//                 {
//                     log_i("Connected to server");
//                 }
//             }

//             PamiReportMessage report = PamiReportMessage(false, 0.0f, PlayingSide::BLUE_SIDE, 2.0f, 42);
//             uint32_t sizeOfMessage = report.serialize((float*) write_buffer, WRITE_BUFFER_SIZE / 4);

//             size_t sizeOfSentMessage = wifiClient.write_P((char*) write_buffer, sizeOfMessage * 4);

//             if (sizeOfSentMessage != sizeOfMessage * 4)
//             {
//                 log_e("Failed to send report to SCD");
//             }   
//             else
//             {
//                 // Wait for reply
//                 long startMillis = millis();
//                 while (wifiClient.connected() && !wifiClient.available() && (millis() - startMillis < RESPONSE_TIMEOUT))
//                 {
//                     vTaskDelay(10 / portTICK_PERIOD_MS);
//                 }
                
//                 if (wifiClient.available())
//                 {
//                     uint32_t len = wifiClient.read((uint8_t *)read_buffer, READ_BUFFER_SIZE);
//                     log_d("Sent report and received message size: %d", len);
//                 }
//                 else
//                 {
//                     log_e("Server response timeout");
//                 }
//             }
            
//             vTaskDelay(500 / portTICK_PERIOD_MS);
//         }
//     }

//     void init()
//     {
//         xTaskCreate(
//             task_report_broadcast,
//             "task_report_broadcast",
//             30000,
//             NULL,
//             60,
//             NULL);
//     }
// }
