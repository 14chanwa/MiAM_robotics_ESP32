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
#include <FastLED.h>
#include <vlx_sensor_and_display.hpp>


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

VLXSensorAndDisplay vlx_sensor_and_display(0, 1);

void setup()
{
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

  FastLED.setBrightness(10);
  Wire.begin(sda, scl, frequency); //This resets to 100kHz I2C
  //Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 

  bool init_vlx = vlx_sensor_and_display.init();
  if (init_vlx)
  {
    Serial.println("VLX init OK");
  }
  else
  {

    Serial.println("VLX init failed");
  }
  delay(100);
}


void loop()
{
  
  vlx_sensor_and_display.update();
  FastLED.show();

  delay(10); //Small delay between polling
}