#include <Arduino.h>
#include <LEDTimer.hpp>
#include <vector>
#include <memory>
#include <FastLED.h>

#include <WiFi.h>
#include <esp_now.h>
// Communication protocol
// Define a data structure
#define COM_NUM_LED 6
typedef struct struct_message {
  uint8_t red[COM_NUM_LED];
  uint8_t green[COM_NUM_LED];
  uint8_t blue[COM_NUM_LED];
  uint8_t brightness;
} struct_message;

// Create a structured object
struct_message myData;

#define LED_PIN 2
#define LED_PWM_CHANNEL 0

#define LED_PWM_FREQUENCY 1000
#define LED_PWM_RESOLUTION 8
#define LED_PWM_HIGH_LEVEL 32

// LED RING
#define LED_RING_BRIGHTNESS 150
#define LED_WARTHOG_BRIGHTNESS 20
#define FASTLED_PIN 13
#define FASTLED_WARTHOG_PIN 12
#define NUM_LEDS COM_NUM_LED

#define BAORD_LED_PIN 21

CRGB leds[NUM_LEDS];
char strip_brightness = LED_RING_BRIGHTNESS;
CRGB leds_warthog[2];

CRGB led_board[1];


std::vector<std::shared_ptr<CRGBLEDTimer > > side_indicator_ring_timers;
PWMLEDTimer status_led_timer(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);

void task_blink_led(void* parameters)
{
  for(;;)
  {
    status_led_timer.update();

    for (auto timer : side_indicator_ring_timers)
    {
        timer->update();
    }

    FastLED[0].showLeds(strip_brightness);
    FastLED[1].showLeds(LED_WARTHOG_BRIGHTNESS);
    FastLED[2].showLeds(strip_brightness);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // LED strip timers
  for (int i=0; i<NUM_LEDS; i++)
  {
    side_indicator_ring_timers.at(i)->setColor(CRGB(myData.red[i],myData.green[i],myData.blue[i]));
  }
  led_board[0] = CRGB(myData.red[0],myData.green[0],myData.blue[0]);
}


void setup()
{
  FastLED.addLeds<WS2812B, FASTLED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, FASTLED_WARTHOG_PIN, GRB>(leds_warthog, 2);  // GRB ordering is typical
  FastLED.addLeds<WS2812, BAORD_LED_PIN, GRB>(led_board, 1);  // GRB ordering is typical
  leds_warthog[0] = CRGB::Red;
  leds_warthog[1] = CRGB::Red;

  
  ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
  ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION);

  // Back LED and status led
  status_led_timer.setPeriod(500);

  // LED strip timers
  for (int i=0; i<NUM_LEDS; i++)
  {
    side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[i])));
    side_indicator_ring_timers.at(i)->setPeriod(0);
    side_indicator_ring_timers.at(i)->setColor(CRGB::Green);
    led_board[0] = CRGB::Green;
  }

  Serial.begin(115200);
  delay(500);
  xTaskCreate(
    task_blink_led,
    "task_blink_led",
    10000,
    NULL,
    50,
    NULL
  );

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

}

void loop()
{

}