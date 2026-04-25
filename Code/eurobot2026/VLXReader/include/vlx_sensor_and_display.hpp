#ifndef VLX_SENSOR_AND_DISPLAY_H
#define VLX_SENSOR_AND_DISPLAY_H

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <FastLED.h>

class VLXSensorAndDisplay
{
public:
    SparkFun_VL53L5CX* vlx_sensor;
    VL53L5CX_ResultsData measurement_data;
    uint8_t i2c_channel;

    uint8_t led_data_pin;
    CRGB* leds;
    
    int imageResolution;
    int imageWidth;

    VLXSensorAndDisplay(
        uint8_t i2c_channel_,
        uint8_t led_data_pin_
    ) ;

    bool init();
    bool update();
};


#endif