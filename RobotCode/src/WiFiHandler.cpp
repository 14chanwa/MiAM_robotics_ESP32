#include <WiFiHandler.hpp>
#include <secret.hpp>
#include <parameters.hpp>

#include <esp_wifi.h>

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

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
}

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

        WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
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
}