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
#include <PacketSerial.h>

#include <vlx_sensor.hpp>
#include <secret.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

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
#define LEDS_3_PIN 4

long last_time_blink = 0;
bool debug_led_state = false;

CRGB led_debug[1];
CRGB leds_0[NUM_LEDS*2];
CRGB leds_1[2];
CRGB leds_2[1];
CRGB leds_3[2];

uint32_t last_ota_time = 0;

VLXSensor vlx_sensor_0(0);
VLXSensor vlx_sensor_1(1);

#define SERIAL_PRINT(x)
#define SERIAL_PRINTF(x, y)
#define SERIAL_PRINTLN(x)

// Com with rpi
PacketSerial myPacketSerial;
uint8_t buffer[100];


bool res;
bool need_send = false;

void onPacketReceived(const uint8_t* buffer, size_t size)
{

}

class NearestPointData
{
public:
  int distance_mm;
  float angle;
  NearestPointData(int d_, float angle_): distance_mm(d_), angle(angle_) {}

  bool operator<(const NearestPointData other)
  {
    return distance_mm < other.distance_mm;
  }
};
NearestPointData nearest_distance(3000, 0);
NearestPointData pointToSend(DISTANCE_MAX, 0);


// Coordinates of center of vlx in frame of robot
// Right VLX (0)
#define XRIGHT 150
#define YRIGHT -60

// Left VLX (1)
#define XLEFT 150
#define YLEFT 60

NearestPointData vlx_to_robot_frame(NearestPointData vlxDataPoint, int sensor_id)
{
  float r = vlxDataPoint.distance_mm;
  float theta = vlxDataPoint.angle;
  float X = 0;
  float Y = 0;
  if (sensor_id == 0)
  {
    X = XRIGHT;
    Y = YRIGHT;
  }
  else
  {
    X = XLEFT;
    Y = YLEFT;
  }

  float xhat = X + r * cos(theta);
  float yhat = Y + r * sin(theta);

  float R = sqrt(
    X*X + 2*X*r*cos(theta) + Y*Y + 2*Y*sin(theta) + r*r
  );
  float THETA = atan2(Y + r*sin(theta), X + r*cos(theta));

  return NearestPointData(R, THETA);
}


NearestPointData update_leds_from_data(VL53L5CX_ResultsData measurement_data, CRGB* crgb_leds, int sensor_id)
{
    // Update nearest distance
    std::vector<NearestPointData > distances;
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
                SERIAL_PRINT("Indice invalide: ");
                SERIAL_PRINTLN(led_index);
                continue;
            }
            SERIAL_PRINT("\t");
            if (measurement_data.target_status[x + y] == 5 || measurement_data.target_status[x + y] == 9)
            {
                int distance = measurement_data.distance_mm[x + y];

                // Row index starting bottom of image
                int yprime = int(floor(y / imageWidth));

                // Ignore bottom rows: possibly elephant
                if ((yprime > OFFSET_ARM_ROWS) || (distance >= DISTANCE_NEAR))
                {
                    // compute angle 
                    float angle_deg = 0;
                    angle_deg = -60 + x * 17;
                    // if (sensor_id == 0)
                    // {
                    //   switch(x)
                    //   {
                    //     case 0:
                    //       angle = -60;
                    //       break;
                    //     case 1:
                    //       angle = -45;
                    //       break;
                    //     case 2:
                    //       angle = -30;
                    //       break;
                    //     default:
                    //       angle = 0;
                    //       break;
                    //   }
                    //   // Index 0 is -60°
                    //   // Index 1 is -45°...
                    //   // Index 7 is 45°
                    //   //angle = -60 + 15 * x;
                    // }
                    // else
                    // {
                    //   switch(x)
                    //   {
                    //     case 7:
                    //       angle = 60;
                    //       break;
                    //     case 6:
                    //       angle = 45;
                    //       break;
                    //     case 5:
                    //       angle = 30;
                    //       break;
                    //     default:
                    //       angle = 0;
                    //       break;
                    //   }
                      // Index 0 is -4°
                      // Index 1 is -12°...
                      // Index 7 is 60°
                      //angle = -4 + 8 * x;
                    //}
                    distances.push_back(
                      vlx_to_robot_frame(
                        NearestPointData(
                          distance, 
                          angle_deg * M_TWOPI / 360.0
                        ), 
                        sensor_id
                      )
                    );
                }

                SERIAL_PRINT(distance);
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
              SERIAL_PRINT("*");
              crgb_leds[led_index] = CRGB::Black;
          }
          
        }
        SERIAL_PRINTLN();
    }
    SERIAL_PRINTLN();

    // Update nearest distance
    // Only update if 3 points are resolved
    NearestPointData nearest_distance(3000, 0);
    if (distances.size() >= 3)
    {
        std::sort(distances.begin(), distances.end());
        if (distances.at(2) < nearest_distance)
        {
            nearest_distance = NearestPointData(
              (distances.at(0).distance_mm + 
              distances.at(1).distance_mm + 
              distances.at(2).distance_mm) / 3.0,
              (distances.at(0).angle + 
              distances.at(1).angle + 
              distances.at(2).angle) / 3.0
            );
            //nearest_distance = distances.at(2);
        }
    }
    return nearest_distance;
}


void update_nearest_distance_led(NearestPointData nearest_distance)
{
    // set the proximity led to the right color
    if (nearest_distance.distance_mm < DISTANCE_NEAR)
    {
      leds_2[0] = CRGB::Red;
      leds_2[0].nscale8_video(10);
    }
    else if (nearest_distance.distance_mm < DISTANCE_MID)
    {
      leds_2[0] = CRGB::Orange;
      leds_2[0].nscale8_video(10);
    }
    else if (nearest_distance.distance_mm < DISTANCE_FAR)
    {
      leds_2[0] = CRGB::Green;
      leds_2[0].nscale8_video(10);
    }
    else
    {
      leds_2[0] = CRGB::Black;
    }
}


void lowLevelLoop(void* parameters)
{
    for (;;)
    {
        res = false;
        nearest_distance = NearestPointData(DISTANCE_MAX, 0);
        
        if (vlx_sensor_0.update())
        {
            NearestPointData dist = update_leds_from_data(vlx_sensor_0.measurement_data, leds_0, 0);
            if (dist < nearest_distance)
            {
              nearest_distance = dist;
              update_nearest_distance_led(nearest_distance);
            }
            res |= true;  
        }

        if (vlx_sensor_1.update())
        {
            NearestPointData dist = update_leds_from_data(vlx_sensor_1.measurement_data, &(leds_0[NUM_LEDS]), 1);
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

        // Send if new res
        if (res)
        {
          pointToSend = nearest_distance;
          need_send = true;
        }
        delay(10);
    }
    
}

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
      SERIAL_PRINTLN("Start updating " + type);
    })
    .onEnd([]() {
      SERIAL_PRINTLN("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        SERIAL_PRINTF("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      SERIAL_PRINTF("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        SERIAL_PRINTLN("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        SERIAL_PRINTLN("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        SERIAL_PRINTLN("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        SERIAL_PRINTLN("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        SERIAL_PRINTLN("End Failed");
      }
    });

    ArduinoOTA.begin();


    uint8_t sda = 8;
    uint8_t scl = 9;
    uint32_t frequency = 400000;

    //Serial.begin(115200);
    SERIAL_PRINTLN("MiAM!!!");

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
        SERIAL_PRINTLN("VLX init OK");
    }
    else
    {

        SERIAL_PRINTLN("VLX init failed");
    }

    init_vlx = vlx_sensor_1.init();
    if (init_vlx)
    {
        SERIAL_PRINTLN("VLX init OK");
    }
    else
    {
        SERIAL_PRINTLN("VLX init failed");
    }


    // Setup leds
    FastLED.addLeds<WS2812, 21, GRB>(led_debug, 1);
    FastLED.addLeds<WS2812, LEDS_0_PIN, GRB>(leds_0, NUM_LEDS*2);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_1_PIN, GRB>(leds_1, 2);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_2_PIN, GRB>(leds_2, 1);  // GRB ordering is typical
    FastLED.addLeds<WS2812, LEDS_3_PIN, GRB>(leds_3, 2);  // GRB ordering is typical

    // Set decorative leds
    for (uint i=0; i<2; i++)
    {
        leds_1[i] = CRGB::Pink;
        leds_1[i].nscale8_video(140);
        leds_3[i] = CRGB::Red;
        leds_3[i].nscale8_video(50);
    }

    // Launch update leds task
    xTaskCreatePinnedToCore(
      lowLevelLoop,
      "lowLevelLoop",
      200000,
      NULL,
      10,
      NULL,
      0
    );

    // Begin packetserial
    myPacketSerial.begin(1000000);
    myPacketSerial.setPacketHandler(&onPacketReceived);

    delay(100);
}

void loop()
{

    if (need_send)
    {
      // Send nearest distance over serial
      int32_t valeur_test = pointToSend.distance_mm;
      float angle_test = pointToSend.angle;
      memcpy(&(buffer[0]), &valeur_test, sizeof(int32_t));
      memcpy(&(buffer[sizeof(int32_t)]), &angle_test, sizeof(angle_test));
      myPacketSerial.send(buffer, sizeof(int32_t)+sizeof(float));

      // Serial.print(">r:");
      // Serial.println(pointToSend.distance_mm);
      // Serial.print(">theta:");
      // Serial.println(pointToSend.angle);

      need_send = false;
    }
    ArduinoOTA.handle();
    myPacketSerial.update();

    delay(10); // Small delay between polling
}