#include <WiFiHandler.hpp>
#include <esp_wifi.h>
#include <esp_mac.h>

namespace wifi_handler
{
    String ssid_;
    String password_;
    uint8_t newMacAddress_[6];
    bool newMacAddressSet_ = false;

    void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Connected to AP successfully!");
    }

    void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Got new IP address");
        Serial.println(WiFi.localIP());
    }

    void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Disconnected from WiFi access point");
        log_i("WiFi lost connection. Reason: %d", info.wifi_sta_disconnected.reason);
        log_i("Trying to Reconnect");
        WiFi.begin(ssid_, password_);
    }

    void setBaseMACAddress(const uint8_t mac[6])
    {
        for (uint8_t i=0; i<6; i++)
        {
            newMacAddress_[i] = mac[i];
        }
        newMacAddressSet_ = true;

        // Need to set this for BT
        esp_err_t err = esp_base_mac_addr_set(&newMacAddress_[0]);

        uint8_t newMac[6];
        esp_read_mac(&newMac[0], ESP_MAC_WIFI_STA);
        if (err == ESP_OK)
        {
            log_i("MAC address successfully set to %02x:%02x:%02x:%02x:%02x:%02x", newMac[0], newMac[1], newMac[2], newMac[3], newMac[4], newMac[5]);
        }
        else
        {
            log_e("Failed to set new MAC Address");
        }
        
        // Let it sink
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    void printCurrentMACAddress()
    {
        // Get the MAC address of the Wi-Fi station interface
        uint8_t baseMac[6];
        
        // Get MAC address for WiFi station
        esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
        Serial.print("Station MAC: ");
        for (int i = 0; i < 5; i++) {
            Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n", baseMac[5]);
        
        // Get the MAC address of the Wi-Fi AP interface
        esp_read_mac(baseMac, ESP_MAC_WIFI_SOFTAP);
        Serial.print("SoftAP MAC: ");
        for (int i = 0; i < 5; i++) {
            Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n", baseMac[5]);
        
        // Get the MAC address of the Bluetooth interface
        esp_read_mac(baseMac, ESP_MAC_BT);
        Serial.print("Bluetooth MAC: ");
        for (int i = 0; i < 5; i++) {
            Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n", baseMac[5]);

        // Get the MAC address of the Ethernet interface
        esp_read_mac(baseMac, ESP_MAC_ETH);
        Serial.print("Ethernet MAC: ");
        for (int i = 0; i < 5; i++) {
            Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n", baseMac[5]);
        
        Serial.println();
    }

    void setOTAOnStart(std::function<void(void)> fn)
    {
        ArduinoOTA.onStart(fn);
    }

    void task_handle_ota(void *parameters)
    {
        for (;;)
        {
            ArduinoOTA.handle();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }

    void connectSTA(String ssid, String password)
    {
        WiFi.mode(WIFI_STA);

        if (newMacAddressSet_)
        {   
            esp_wifi_set_mac(WIFI_IF_STA, &newMacAddress_[0]);
        }

        ssid_ = ssid;
        password_ = password;

        WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
        WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
        WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

        Serial.print("Current WiFi MAC Address: ");
        Serial.println(WiFi.macAddress());

        WiFi.begin(ssid_, password_);
        Serial.print("Connecting to WiFi ..");
        while (WiFi.status() != WL_CONNECTED)
        {
            Serial.print('.');
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        Serial.println();
        Serial.print("Current ESP32 IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Gateway (router) IP: "); 
        Serial.println(WiFi.gatewayIP());
        Serial.print("Subnet Mask: "); 
        Serial.println(WiFi.subnetMask());
        Serial.print("Primary DNS: ");
        Serial.println(WiFi.dnsIP(0));
        Serial.print("Secondary DNS: "); 
        Serial.println(WiFi.dnsIP(1));

        ArduinoOTA.begin();

        xTaskCreate(
            task_handle_ota,
            "task_handle_ota",
            10000,
            NULL,
            70,
            NULL);
    }
}