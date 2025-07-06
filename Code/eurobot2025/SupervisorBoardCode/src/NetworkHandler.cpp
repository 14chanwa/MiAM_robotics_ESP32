#include <NetworkHandler.hpp>
//#include <secret.hpp>

// #define USE_WIFI
#define ENABLE_OTA_UPDATE

#ifdef USE_WIFI
    #include <WiFi.h>
#else
    #include <ETH.h>
    /* 
    * ETH_CLOCK_GPIO0_IN   - default: external clock from crystal oscillator
    * ETH_CLOCK_GPIO0_OUT  - 50MHz clock from internal APLL output on GPIO0 - possibly an inverter is needed for LAN8720
    * ETH_CLOCK_GPIO16_OUT - 50MHz clock from internal APLL output on GPIO16 - possibly an inverter is needed for LAN8720
    * ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
    */
    #define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
    // Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
    #define ETH_POWER_PIN   16
    // Type of the Ethernet PHY (LAN8720 or TLK110)
    #define ETH_TYPE        ETH_PHY_LAN8720
    // I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
    #define ETH_ADDR        1
    // Pin# of the I²C clock signal for the Ethernet PHY
    #define ETH_MDC_PIN     23
    // Pin# of the I²C IO signal for the Ethernet PHY
    #define ETH_MDIO_PIN    18
#endif


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


namespace NetworkHandler
{
    void init() {
#ifdef USE_WIFI
        WiFi.mode(WIFI_STA);

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
#else
        ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
        Serial.print("ETH begins on MAC address: ");
        Serial.println(ETH.macAddress());
#endif

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
    }


    IPAddress localIP()
    {
#ifdef USE_WIFI
        return WiFi.localIP();
#else
        return ETH.localIP();
#endif
    }
}