#ifndef _TASKS_HEADER
#define _TASKS_HEADER

#include <parameters.hpp>
#include <Arduino.h>
#include "WString.h"
#include <Wire.h>

/////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////

// blink led
void task_blink_led(void* parameters);

// monitor battery

void monitor_battery();
void print_battery();
float get_current_battery_reading();
float get_min_battery_reading();

float get_current_tcrt0_reading();
float get_current_tcrt1_reading();
float get_current_tcrt2_reading();
float get_current_touch_sensor_reading();

void task_update_analog_readings(void* parameters);

void init_vl53l0x(TwoWire* wire);
uint16_t get_current_vl53l0x();
void task_update_vl53l0x(void* parameters);
void update_vl53l0x();

void initOLEDScreen(TwoWire* wire);
void printOLEDMessage(String message);
void task_update_ssd1306(void* parameters);

class DisplayInformations
{
    public:
        DisplayInformations()
        {
            ip_address = new char[16]();
        };
        char* ip_address;
        int id;
};

void update_ssd1306(DisplayInformations* informations);

// // handle encoders
// void run_handle_encoders();
// int get_encoder_values(unsigned char id);
// int get_encoder_speed(unsigned char id);
// void task_print_encoder(void* parameters);
// void task_get_encoder_speed(void* parameters);

// // low level loop

// void run_low_level_loop(RobotWheelDC* motor);
// void task_low_level_loop(void* parameters);
// void task_print_low_level(void* parameters);

#endif