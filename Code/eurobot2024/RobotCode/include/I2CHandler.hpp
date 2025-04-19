#ifndef _I2C_HANDLER
#define _I2C_HANDLER

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
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
    enum Side
    {
        RIGHT,
        MIDDLE,
        LEFT
    };

    void init();
    TwoWire* get_wire();

    bool i2c_get();
    bool i2c_give();

    void init_vl53l0x();
    uint16_t get_current_vl53l0x();
    uint16_t get_smoothed_vl53l0x();
    int16_t get_smoothed_vlx_side(Side side);
    void update_vl53l0x();

    uint16_t get_bottom_smoothed();

    // void initOLEDScreen();
    // void printOLEDMessage(String message);

    // void update_ssd1306(DisplayInformations* informations);
};

class VLXSensor
{
public:
    VLXSensor() {}
    virtual void init(uint8_t target_i2c_addr) = 0;

    uint16_t get_current() { return current_; }
    uint16_t get_smoothed() { return smoothed_; }

    virtual void update() = 0;
    bool is_init() { return is_init_; }
protected:
    bool is_init_ = false;
    ADCReading adc_;
    uint16_t current_ = 8000;
    uint16_t smoothed_ = 8000;
};

class VL0XSensor: public VLXSensor
{
public:
    VL0XSensor() : VLXSensor() {}
    void init(uint8_t target_i2c_addr);
    void update();

private:
    Adafruit_VL53L0X lox_;
};

class VL1XSensor: public VLXSensor
{
public:
    VL1XSensor() : VLXSensor() {}
    void init(uint8_t target_i2c_addr);
    void update();

private:
    Adafruit_VL53L1X l1x_;
};

#endif