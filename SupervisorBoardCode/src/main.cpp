/****************************************************************************************************************************
  UDPSendReceive.ino - Simple Arduino web server sample for ESP8266/ESP32 AT-command shield

  For Ethernet shields using WT32_ETH01 (ESP32 + LAN8720)

  WebServer_WT32_ETH01 is a library for the Ethernet LAN8720 in WT32_ETH01 to run WebServer

  Based on and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_WT32_ETH01
  Licensed under MIT license
 *****************************************************************************************************************************/

#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial

// Debug Level from 0 to 4
#define _ETHERNET_WEBSERVER_LOGLEVEL_       3

#include <WebServer_WT32_ETH01.h>
#include <TFTScreen.hpp>
#include <esp_wifi.h>
#include <ArduinoOTA.h>

#define SET_SIDE_PIN 39
#define FUNCTION_PIN 36
#define START_SWITCH_PIN 35

// // Select the IP address according to your local network
// IPAddress myIP(192, 168, 2, 232);
// IPAddress myGW(192, 168, 2, 1);
// IPAddress mySN(255, 255, 255, 0);

// Google DNS Server IP
IPAddress myDNS(8, 8, 8, 8);

unsigned int localPort = 779;    //10002;  // local port to listen on

char packetBuffer[255];          // buffer to hold incoming packet
byte ReplyBuffer[] = "ACK";      // a string to send back

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// Screen
TFTScreen tftScreen;

void task_update_screen(void* parameters)
{
  for(;;)
  {
    tftScreen.update(ETH.localIP());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Read buttons with hysteresis
bool set_side_pin_state = LOW;
bool function_pin_state = LOW;
unsigned long debounceDelay = 200;
unsigned long last_set_side_change = 0;
unsigned long last_function_change = 0;

void task_read_pins(void* parameters)
{
  pinMode(SET_SIDE_PIN, INPUT);
  pinMode(FUNCTION_PIN, INPUT);
  unsigned long currentTime;
  for(;;)
  {
    currentTime = millis();
    bool new_set_side_pin_state = digitalRead(SET_SIDE_PIN);
    bool new_function_pin_state = digitalRead(FUNCTION_PIN);
    if (currentTime - last_set_side_change > debounceDelay && set_side_pin_state != new_set_side_pin_state)
    {
      set_side_pin_state = new_set_side_pin_state;
      last_set_side_change = currentTime;
      Serial.print("SET_SIDE_PIN value: ");
      Serial.println(set_side_pin_state);
    }
    if (currentTime - last_function_change > debounceDelay && function_pin_state != new_function_pin_state)
    {
      function_pin_state = new_function_pin_state;
      last_function_change = currentTime;
      Serial.print("FUNCTION_PIN value: ");
      Serial.println(function_pin_state);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void task_handle_ota(void* parameters)
{
  for (;;)
  {
    ArduinoOTA.handle();  
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
}

void setup()
{
  // Disable WiFi (don't need it)
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);

  tftScreen.init();
  xTaskCreate(
    task_update_screen,
    "task_update_screen",
    10000,
    NULL,
    10,
    NULL
  );

  xTaskCreate(
    task_read_pins,
    "task_read_pins",
    10000,
    NULL,
    10,
    NULL
  );

  // Using this if Serial debugging is not necessary or not using Serial port
  //while (!Serial && (millis() < 3000));

  Serial.print("\nStarting UDPSendReceive on " + String(ARDUINO_BOARD));
  Serial.println(" with " + String(SHIELD_TYPE));
  Serial.println(WEBSERVER_WT32_ETH01_VERSION);

  // To be called before ETH.begin()
  WT32_ETH01_onEvent();

  //bool begin(uint8_t phy_addr=ETH_PHY_ADDR, int power=ETH_PHY_POWER, int mdc=ETH_PHY_MDC, int mdio=ETH_PHY_MDIO,
  //           eth_phy_type_t type=ETH_PHY_TYPE, eth_clock_mode_t clk_mode=ETH_CLK_MODE);
  //ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);

  // // Static IP, leave without this line to get IP via DHCP
  // //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  // ETH.config(myIP, myGW, mySN, myDNS);

  WT32_ETH01_waitForConnect();

  xTaskCreate(
    task_handle_ota,
    "task_handle_ota",
    10000,
    NULL,
    10,
    NULL
  );

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();



  Serial.println(F("\nStarting connection to server..."));
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  Serial.print(F("Listening on port "));
  Serial.println(localPort);
}


void loop()
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize)
  {
    Serial.print(F("Received packet of size "));
    Serial.println(packetSize);
    Serial.print(F("From "));
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(F(", port "));
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);

    if (len > 0)
    {
      packetBuffer[len] = 0;
    }

    Serial.println(F("Contents:"));
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer, sizeof(ReplyBuffer));
    Udp.endPacket();
  }
}