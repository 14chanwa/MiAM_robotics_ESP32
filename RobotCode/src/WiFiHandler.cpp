#include <WiFiHandler.hpp>
#include <secret.hpp>

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
        WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
        Serial.print("Connecting to WiFi ..");
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print('.');
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        Serial.println(WiFi.localIP());
        vTaskDelay(1000 / portTICK_PERIOD_MS);

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

        // Register callback in case WiFi disconnects
        WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    }
}