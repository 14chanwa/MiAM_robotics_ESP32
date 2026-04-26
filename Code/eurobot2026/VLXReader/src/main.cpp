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
#include <vector>
#include <algorithm>

// How many leds in your strip?
#define NUM_LEDS 64
#define DISTANCE_FAR 1000
#define DISTANCE_MID 500
#define DISTANCE_NEAR 250
#define DISTANCE_MAX 3000

#define OFFSET_ARM_ROWS 3

#define LEDS_0_PIN 1
#define LEDS_1_PIN 2
#define LEDS_2_PIN 3

long last_time_blink = 0;
bool debug_led_state = false;

CRGB led_debug[1];
CRGB leds_0[NUM_LEDS*2];
CRGB leds_1[2];
CRGB leds_2[1];

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
    FastLED.addLeds<WS2812, LEDS_0_PIN, GRB>(leds_0, NUM_LEDS*2);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_1_PIN, GRB>(leds_1, 2);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_2_PIN, GRB>(leds_2, 1);  // GRB ordering is typical

    // Set decorative leds
    for (uint i=0; i<2; i++)
    {
        leds_1[i] = CRGB::Pink;
        leds_1[i].nscale8_video(140);
    }

    delay(100);
}

int update_leds_from_data(VL53L5CX_ResultsData measurement_data, CRGB* crgb_leds)
{
    // Update nearest distance
    std::vector<int > distances;
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

                // Row index starting bottom of image
                int yprime = int(floor(y / imageWidth));

                // Ignore bottom rows: possibly elephant
                if ((yprime > OFFSET_ARM_ROWS) || (distance >= DISTANCE_NEAR))
                {
                    distances.push_back(distance);
                }

                Serial.print(distance);
                if (distance > DISTANCE_FAR)
                {
                    crgb_leds[led_index] = CRGB::Black;
                    // crgb_leds[led_index] = CRGB::Pink;
                    // crgb_leds[led_index].nscale8_video(3);
                }
                else if (distance > DISTANCE_MID)
                {
                    crgb_leds[led_index] = CRGB::Green;
                    crgb_leds[led_index].nscale8_video(4);
                }
                else if (distance > DISTANCE_NEAR)
                {
                    crgb_leds[led_index] = CRGB::Orange;
                    crgb_leds[led_index].nscale8_video(4);
                }
                else
                {
                    if (yprime <= OFFSET_ARM_ROWS)
                    {
                        crgb_leds[led_index] = CRGB::Blue;
                    }
                    else
                    {
                        crgb_leds[led_index] = CRGB::Red;
                    }
                    crgb_leds[led_index].nscale8_video(4);
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

    // Update nearest distance
    // Only update if 3 points are resolved
    int nearest_distance = DISTANCE_MAX;
    if (distances.size() >= 3)
    {
        std::sort(distances.begin(), distances.end());
        if (distances.at(2) < nearest_distance)
        {
            nearest_distance = distances.at(2);
        }
    }
    return nearest_distance;
}

bool res;

void update_nearest_distance_led(int nearest_distance)
{
    // set the proximity led to the right color
    if (nearest_distance < DISTANCE_NEAR)
    {
      leds_2[0] = CRGB::Red;
      leds_2[0].nscale8_video(10);
    }
    else if (nearest_distance < DISTANCE_MID)
    {
      leds_2[0] = CRGB::Orange;
      leds_2[0].nscale8_video(10);
    }
    else if (nearest_distance < DISTANCE_FAR)
    {
      leds_2[0] = CRGB::Green;
      leds_2[0].nscale8_video(10);
    }
    else
    {
      leds_2[0] = CRGB::Black;
    }
}

int nearest_distance;

void loop()
{
    res = false;
    nearest_distance = DISTANCE_MAX;
    
    if (vlx_sensor_0.update())
    {
        int dist = update_leds_from_data(vlx_sensor_0.measurement_data, leds_0);
        if (dist < nearest_distance)
        {
          nearest_distance = dist;
          update_nearest_distance_led(nearest_distance);
        }
        res |= true;  
    }

    if (vlx_sensor_1.update())
    {
        int dist = update_leds_from_data(vlx_sensor_1.measurement_data, &(leds_0[NUM_LEDS]));
        if (dist < nearest_distance)
        {
          nearest_distance = dist;
          update_nearest_distance_led(nearest_distance);
        }
        res |= true;
    }

    // debug led
    if (millis() - last_time_blink > 1000)
    {
      if (debug_led_state)
      {
        led_debug[0] = CRGB::Green;
        led_debug[0].nscale8_video(10);
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