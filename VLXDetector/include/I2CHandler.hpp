#ifndef _I2C_HANDLER
#define _I2C_HANDLER

#include <Arduino.h>
#include <Wire.h>

namespace I2CHandler
{
    void init();
    TwoWire* get_wire();
    TwoWire* get_wire2();

    bool i2c_get();
    bool i2c_give();

    void init_vl53l0x();
    uint16_t get_current_vl53l0x();
    uint16_t get_smoothed_vl53l0x();
    uint16_t get_current_vl53l0x2();
    uint16_t get_smoothed_vl53l0x2();
    void update_vl53l0x();
};

#endif