#ifndef _I2C_HANDLER
#define _I2C_HANDLER

#include <Arduino.h>
#include <Wire.h>

class DisplayInformations
{
    public:
        DisplayInformations()
        {
            ip_address = new char[16]();
        };
        char* ip_address;
        int id;
        bool match_started;
        int current_time_s;
};

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

    void initOLEDScreen();
    void printOLEDMessage(String message);

    void update_ssd1306(DisplayInformations* informations);
};

#endif