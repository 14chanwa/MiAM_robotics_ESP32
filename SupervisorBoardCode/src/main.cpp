
#include <TFTScreen.hpp>
#include <esp_wifi.h>
#include <ArduinoOTA.h>

#include <MessageHandler.hpp>
#include <Button.hpp>
#include <Match.hpp>
#include <MessageReceiver.hpp>

#define USE_WIFI

#ifdef USE_WIFI
#include <WiFiHandler.hpp>
#else
#include <WebServer_WT32_ETH01.h>
#endif

#define SET_SIDE_PIN 36
#define FUNCTION_PIN 39
#define START_SWITCH_PIN 35

// Screen
TFTScreen tftScreen;

// Read buttons with hysteresis
Button set_side_button(SET_SIDE_PIN);
Button function_button(FUNCTION_PIN);
Button start_switch_button(START_SWITCH_PIN);

void task_update_screen(void* parameters)
{
  for(;;)
  {
#ifdef USE_WIFI
    tftScreen.update(WiFi.localIP());
#else
    tftScreen.update(ETH.localIP());
#endif
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void task_read_pins(void* parameters)
{
  set_side_button.init();
  function_button.init();
  start_switch_button.init();
  
  for(;;)
  {
    set_side_button.update();
    function_button.update();
    start_switch_button.update();
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

void task_monitor_buttons(void* parameters)
{
  for (;;)
  {
    // Check buttons
    ButtonEvent buttonEvent;

    // Set side button triggers messages
    buttonEvent = set_side_button.getEvent();
    if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
    {
      // Change color
      if (Match::getSide() == PlayingSide::BLUE_SIDE)
      {
        Match::setSide(PlayingSide::YELLOW_SIDE);
      }
      else
      {
        Match::setSide(PlayingSide::BLUE_SIDE);
      }
    }

    // Function button starts match
    // TODO
    buttonEvent = function_button.getEvent();
    if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
    {
      if (Match::getMatchStarted())
      {
        Match::stopMatch();
      }
      else
      {
        Match::startMatch(85.0);
      }
    }

    // Switch button starts match
    buttonEvent = start_switch_button.getEvent();

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);

  tftScreen.init();
  xTaskCreatePinnedToCore(
    task_update_screen,
    "task_update_screen",
    10000,
    NULL,
    10,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    task_read_pins,
    "task_read_pins",
    10000,
    NULL,
    90,
    NULL,
    0
  );


  xTaskCreatePinnedToCore(
    task_monitor_buttons,
    "task_monitor_buttons",
    10000,
    NULL,
    90,
    NULL,
    0
  );

#ifdef USE_WIFI
  WiFiHandler::initWiFi();
#else
  // Disable WiFi (don't need it)
  WiFi.mode(WIFI_OFF);
   // To be called before ETH.begin()
  WT32_ETH01_onEvent();
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  WT32_ETH01_waitForConnect();
#endif

  xTaskCreatePinnedToCore(
    task_handle_ota,
    "task_handle_ota",
    10000,
    NULL,
    10,
    NULL,
    0
  );

  ArduinoOTA
    .onStart([]() {
      String type;
      // MessageReceiver::stopReceiving();
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

  MessageHandler::startListening();
}


void loop()
{
  taskYIELD();
}