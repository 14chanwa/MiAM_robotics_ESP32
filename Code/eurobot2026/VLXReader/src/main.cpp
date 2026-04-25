/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include <Wire.h>
#include <FastLED.h>

#include <vlx_sensor.hpp>
#include <secret.hpp>

// How many leds in your strip?
#define NUM_LEDS 64
#define DISTANCE_FAR 500
#define DISTANCE_MID 250
#define DISTANCE_NEAR 0

#define LEDS_0_PIN 1
#define LEDS_1_PIN 2

long last_time_blink = 0;
bool debug_led_state = false;

CRGB led_debug[1];
CRGB leds_0[NUM_LEDS];
CRGB leds_1[NUM_LEDS];

uint32_t last_ota_time = 0;

VLXSensor vlx_sensor_0(0);
VLXSensor vlx_sensor_1(1);

void setup()
{

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

    ArduinoOTA.begin();


    uint8_t sda = 8;
    uint8_t scl = 9;
    uint32_t frequency = 400000;

    Serial.begin(115200);
    Serial.println("MiAM!!!");

    // xTaskCreatePinnedToCore(
    //   task_hello_world,
    //   "task_hello_world",
    //   10000,
    //   NULL,
    //   1,          // priorité basse
    //   NULL,
    //   0           // core 0
    // );

    Wire.begin(sda, scl, frequency); //This resets to 100kHz I2C
    //Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 

    bool init_vlx = vlx_sensor_0.init();
    if (init_vlx)
    {
        Serial.println("VLX init OK");
    }
    else
    {

        Serial.println("VLX init failed");
    }

    init_vlx = vlx_sensor_1.init();
    if (init_vlx)
    {
        Serial.println("VLX init OK");
    }
    else
    {
        Serial.println("VLX init failed");
    }


    // Setup leds
    FastLED.addLeds<WS2812, 21, GRB>(led_debug, 1);
    FastLED.addLeds<WS2812, LEDS_0_PIN, GRB>(leds_0, NUM_LEDS);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_1_PIN, GRB>(leds_1, NUM_LEDS);  // GRB ordering is typical
    FastLED.setBrightness(10);

    delay(100);
}

void update_leds_from_data(VL53L5CX_ResultsData measurement_data, CRGB* crgb_leds)
{

    int imageWidth = 8;
    //The ST library returns the data transposed from zone mapping shown in datasheet
    //Pretty-print data with increasing y, decreasing x to reflect reality
    for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
    {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
            uint led_index = x+imageWidth*(imageWidth-1)-y;
            if (led_index >= NUM_LEDS)
            {
                Serial.print("Indice invalide: ");
                Serial.println(led_index);
                continue;
            }
            Serial.print("\t");
            if (measurement_data.target_status[x + y] == 5 || measurement_data.target_status[x + y] == 9)
            {
                int distance = measurement_data.distance_mm[x + y];
                Serial.print(distance);
                if (distance > DISTANCE_FAR)
                {
                    crgb_leds[led_index] = CRGB::Green;
                }
                else if (distance > DISTANCE_MID)
                {
                    crgb_leds[led_index] = CRGB::Orange;
                }
                else
                {
                    crgb_leds[led_index] = CRGB::Red;
                }
                // else if (distance > DISTANCE_MID)
                // {
                //   crgb_leds[x + y] = (DISTANCE_FAR-distance) * CRGB::Orange + (distance) * CRGB::Green;
                // }
                // else
                // {
                //   crgb_leds[x + y] = (DISTANCE_MID-distance) * CRGB::Red + (distance) * CRGB::Orange;
                // }
          }
          else
          {
              Serial.print("*");
              crgb_leds[led_index] = CRGB::Black;
          }
          
        }
        Serial.println();
    }
    Serial.println();
}

bool res;

void loop()
{

    res = false;
    
    if (vlx_sensor_0.update())
    {
        update_leds_from_data(vlx_sensor_0.measurement_data, leds_0);
        res |= true;  
    }

    if (vlx_sensor_1.update())
    {
        update_leds_from_data(vlx_sensor_1.measurement_data, leds_1);
        res |= true;
    }

    // debug led
    if (millis() - last_time_blink > 1000)
    {
      if (debug_led_state)
      {
        led_debug[0] = CRGB::Green;
      }
      else
      {
        led_debug[0] = CRGB::Black;
      }
      debug_led_state = !debug_led_state;
      last_time_blink = millis();
      res |= true;
    }

    if (res)
    {
      FastLED.show();
    }

    ArduinoOTA.handle();
    delay(20); // Small delay between polling
}