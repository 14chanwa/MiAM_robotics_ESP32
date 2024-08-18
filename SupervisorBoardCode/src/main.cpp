
#include <TFTScreen.hpp>
// #include <esp_wifi.h>

#include <MessageHandler.hpp>
#include <Button.hpp>
#include <Match.hpp>
#include <MessageReceiver.hpp>

#include <ServoHandler.hpp>

// #define USE_WIFI
// #define USE_ARDUINO_OTA

#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiHandler.hpp>
#else
// #include <WebServer_WT32_ETH01.h>
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
  char counter = 0;
  for(;;)
  {
    counter = (counter+1) % 2;
    tftScreen.registerTouch();
    if (counter == 0)
    {
#ifdef USE_WIFI
      tftScreen.update(WiFi.localIP());
#else
      tftScreen.update(ETH.localIP());
#endif
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
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

#ifdef USE_ARDUINO_OTA
#include <ArduinoOTA.h>
void task_handle_ota(void* parameters)
{
  for (;;)
  {
    ArduinoOTA.handle();  
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
}
#endif

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

    // Function button toggles pami motor lock
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
      // Match::setStopMotors(!Match::getStopMotors());
    }

    // Switch button starts match
    buttonEvent = start_switch_button.getEvent();
    if (buttonEvent == ButtonEvent::NEW_STATE_HIGH)
    {
        Match::startMatch(0.0f);
    }
    else if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
    {
        Match::stopMatch();
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void task_handle_servo(void* parameters)
{
    ServoHandler::init();
    bool state = false;
    for (;;)
    {
      if (Match::getMatchStarted() &&
        Match::getMatchTimeSeconds() >= 90 && 
        Match::getMatchTimeSeconds() < 100)
        {
          if (state)
          {
            ServoHandler::servoUp();
            Serial.println("Servo down");
          }
          else
          {
            ServoHandler::servoDown();
            Serial.println("Servo up");
          }
          state = !state;
        }
        else if (Match::getMatchStarted() && Match::getMatchTimeSeconds() >= 100)
        {
            ServoHandler::servoUp();
        }
        else
        {
          ServoHandler::servoFolded();
        }
      vTaskDelay(500 / portTICK_PERIOD_MS);
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
    6,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    task_read_pins,
    "task_read_pins",
    10000,
    NULL,
    8,
    NULL,
    0
  );


  xTaskCreatePinnedToCore(
    task_monitor_buttons,
    "task_monitor_buttons",
    10000,
    NULL,
    7,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    task_handle_servo,
    "task_handle_servo",
    10000,
    NULL,
    1,
    NULL,
    0
  );

#ifdef USE_WIFI
  WiFiHandler::initWiFi();
#else
  // Disable WiFi (don't need it)
  // WiFi.mode(WIFI_OFF);
  
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  Serial.println(ETH.macAddress());
  // WT32_ETH01_waitForConnect();
#endif

#ifdef USE_ARDUINO_OTA

  xTaskCreatePinnedToCore(
    task_handle_ota,
    "task_handle_ota",
    10000,
    NULL,
    30,
    NULL,
    1
  );

  ArduinoOTA
    .onStart([]() {
      String type;
      MessageReceiver::stopReceiving();
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
#endif

  MessageHandler::startListening();
}


void loop()
{
  taskYIELD();
}