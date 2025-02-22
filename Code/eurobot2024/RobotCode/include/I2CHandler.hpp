#ifndef _I2C_HANDLER
#define _I2C_HANDLER

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <ADCReading.hpp>

// class DisplayInformations
// {
//     public:
//         DisplayInformations()
//         {
//             ip_address = new char[16]();
//         };
//         char* ip_address;
//         int id;
//         bool match_started;
//         int current_time_s;
// };

namespace I2CHandler
{
    void init();
    TwoWire* get_wire();

    bool i2c_get();
    bool i2c_give();

    void init_vl53l0x();
    uint16_t get_current_vl53l0x();
    uint16_t get_smoothed_vl53l0x();
    void update_vl53l0x();

    // void initOLEDScreen();
    // void printOLEDMessage(String message);

    // void update_ssd1306(DisplayInformations* informations);
};

class VLXSensor
{
public:
    VLXSensor() {}
    void init(uint8_t target_i2c_addr);

    uint16_t get_current() { return current_; }
    uint16_t get_smoothed() { return smoothed_; }

    void update();
private:
    Adafruit_VL53L0X lox_;
    ADCReading adc_;
    uint16_t current_ = 8000;
    uint16_t smoothed_ = 8000;
};

#endif