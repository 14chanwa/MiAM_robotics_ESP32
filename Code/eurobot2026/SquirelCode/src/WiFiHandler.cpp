#include <WiFiHandler.hpp>
#include <secret.hpp>
#include <parameters.hpp>

#include <esp_wifi.h>

#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 

#define USE_WIFI_CLIENT_API
#define USE_TIMEOUT

 #ifdef ENABLE_OTA_UPDATE
#include <ArduinoOTA.h>

void task_handle_ota(void* parameters)
{
for(;;)
{
    ArduinoOTA.handle();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
}
#endif

void WiFiStationDisconntask_handle_otaected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// #ifdef USE_WIFI_CLIENT_API
// WiFiClient wifiClient;
// #endif

namespace WiFiHandler
{
   
    void initWiFi() {
        WiFi.mode(WIFI_STA);

        Serial.print("[OLD] ESP32 Board MAC Address:  ");
        Serial.println(WiFi.macAddress());
        
        // ESP32 Board add-on before version < 1.0.5
        //esp_wifi_set_mac(ESP_IF_WIFI_STA, &newMACAddress[0]);
        
        // ESP32 Board add-on after version > 1.0.5
        uint8_t newMACAddress[] = WIFI_MAC_ADDRESS;
        esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
        
        Serial.print("[NEW] ESP32 Board MAC Address:  ");
        Serial.println(WiFi.macAddress());

        // // Configure WiFi with static ip
        // if (!WiFi.config(WIFI_LOCAL_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_PRIMARY_DNS, WIFI_SECONDARY_DNS))
        // {
        //     Serial.println("Failed to configure WiFi");
        // }

        WiFi.begin(WIFI_SSID, WIFI_PASS);
        Serial.print("Connecting to WiFi ..");
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print('.');
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        Serial.println();
        Serial.print("Current ESP32 IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Gateway (router) IP: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("Subnet Mask: " );
        Serial.println(WiFi.subnetMask());
        Serial.print("Primary DNS: ");
        Serial.println(WiFi.dnsIP(0));
        Serial.print("Secondary DNS: ");
        Serial.println(WiFi.dnsIP(1));
        // Serial.println(WiFi.localIP());
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        #ifdef ENABLE_OTA_UPDATE
        ArduinoOTA.begin();

        xTaskCreate(
            task_handle_ota,
            "task_handle_ota",
            10000,
            NULL,
            30,
            NULL
        );
        #endif

        // // Register callback in case WiFi disconnects
        // WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    }


//     bool sendTCPMessage(char* message, uint message_size, IPAddress ip_addr, uint port)
//     {
// #ifdef USE_WIFI_CLIENT_API

//     #ifdef USE_TIMEOUT
//         wifiClient.setTimeout(1);
//     #endif

//         // Connect to client
//         if (wifiClient.connect(ip_addr, port))
//         {
//             size_t sizeOfSentMessage = wifiClient.write_P(message, message_size);
//             wifiClient.stop();
//             if (sizeOfSentMessage == message_size)
//                 return true;
//         }
//         return false; 
// #else
//         // creating socket 
//         int clientSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
//         // Serial.print("Address: ");
//         // Serial.println(ip_addr.toString().c_str());

//         // specifying the address 
//         sockaddr_in serverAddress; 
//         serverAddress.sin_family = AF_INET; 
//         serverAddress.sin_port = htons(port); 
//         serverAddress.sin_addr.s_addr = inet_addr(ip_addr.toString().c_str());

//         // connect to server
//         if (connect(clientSocket, (struct sockaddr*)&serverAddress, 
//             sizeof(serverAddress)))
//         {
//             // Serial.println("Connected");
//             // sending data 
//             size_t sizeOfSentMessage = send(clientSocket, message, message_size, 0); 

//             if (sizeOfSentMessage == message_size)
//             {
//                 return true;
//             }
            
//             // closing socket 
//             close(clientSocket);            
//         }

//         return false;
// #endif
//     }
}