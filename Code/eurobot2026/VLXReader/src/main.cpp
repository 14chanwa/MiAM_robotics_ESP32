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
#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 64
#define PIN_DATA 1

// Define the array of leds
CRGB leds[NUM_LEDS];

#define DISTANCE_FAR 500
#define DISTANCE_MID 250
#define DISTANCE_NEAR 0

void switch_i2c(int channel) {

  // Si le numéro de voie est correct, alors on change "l'aiguillage interne du TCA9548" pour connecter cette sortie à l'entrée
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


void task_hello_world(void *)
{
  while(true)
  {
    Serial.println("Hello");
    delay(1000);
    Serial.println("World");
    delay(1000);
  }
}


void setup()
{
  uint8_t sda = 8;
  uint8_t scl = 9;
  uint32_t frequency = 400000;

  Serial.begin(115200);
  Serial.println("MiAM!!!");

  xTaskCreate(
    task_hello_world,
    "task_hello_world",
    10000,
    NULL,
    80,
    NULL
  );

  FastLED.addLeds<WS2812, PIN_DATA, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(10);

  Wire.begin(sda, scl, frequency); //This resets to 100kHz I2C
  //Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 

  switch_i2c(0);
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin(0x29, Wire) == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  myImager.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  //Using 4x4, min frequency is 1Hz and max is 60Hz
  //Using 8x8, min frequency is 1Hz and max is 15Hz
  bool response = myImager.setRangingFrequency(15);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1) ;
  }


  // Set the ranging mode
  response = myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  if (response == true)
  {
    SF_VL53L5CX_RANGING_MODE mode = myImager.getRangingMode();
    switch (mode)
    {
      case SF_VL53L5CX_RANGING_MODE::AUTONOMOUS:
        Serial.println(F("Ranging mode set to autonomous."));
        break;

      case SF_VL53L5CX_RANGING_MODE::CONTINUOUS:
        Serial.println(F("Ranging mode set to continuous."));
        break;

      default:
        Serial.println(F("Error recovering ranging mode."));
        break;
    }
  }
  else
  {
    Serial.println(F("Cannot set ranging mode requested. Freezing..."));
    while (1) ;
  }

  myImager.startRanging();
}


void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          if (measurementData.target_status[x + y] == 5 || measurementData.target_status[x + y] == 9)
          {
            int distance = measurementData.distance_mm[x + y];
            Serial.print(distance);
            if (distance > DISTANCE_FAR)
            {
              leds[x + y] = CRGB::Green;
            }
            else if (distance > DISTANCE_MID)
            {
              leds[x + y] = CRGB::Orange;
            }
            else
            {
              leds[x + y] = CRGB::Red;
            }
            // else if (distance > DISTANCE_MID)
            // {
            //   leds[x + y] = (DISTANCE_FAR-distance) * CRGB::Orange + (distance) * CRGB::Green;
            // }
            // else
            // {
            //   leds[x + y] = (DISTANCE_MID-distance) * CRGB::Red + (distance) * CRGB::Orange;
            // }
          }
          else
          {
            Serial.print("*");
            leds[x + y] = CRGB::Black;
          }
          
        }
        Serial.println();
      }
      Serial.println();
    }
  }
  FastLED.show();

  delay(5); //Small delay between polling
}