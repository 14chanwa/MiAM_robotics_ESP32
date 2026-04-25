#ifndef VLX_SENSOR_AND_DISPLAY_H
#define VLX_SENSOR_AND_DISPLAY_H

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <FastLED.h>

class VLXSensor
{
public:
    SparkFun_VL53L5CX* vlx_sensor;
    VL53L5CX_ResultsData measurement_data;
    uint8_t i2c_channel;
    
    int imageResolution;
    int imageWidth;

    VLXSensor(
        uint8_t i2c_channel_
    ) ;

    bool init();
    bool update();
};


#endif