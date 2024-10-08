#ifndef PS3_DATA_TYPE_H
#define PS3_DATA_TYPE_H

#include <Arduino.h>

namespace ps3_data_type
{
    typedef struct
    {
        int8_t lx;
        int8_t ly;
        int8_t rx;
        int8_t ry;
    } ps3_analog_stick_t;

    typedef struct
    {
        uint8_t up;
        uint8_t right;
        uint8_t down;
        uint8_t left;

        uint8_t l2;
        uint8_t r2;
        uint8_t l1;
        uint8_t r1;

        uint8_t triangle;
        uint8_t circle;
        uint8_t cross;
        uint8_t square;
    } ps3_analog_button_t;

    typedef struct
    {
        ps3_analog_stick_t stick;
        ps3_analog_button_t button;
    } ps3_analog_t;

    /*********************/
    /*   B U T T O N S   */
    /*********************/

    typedef struct
    {
        uint8_t select : 1;
        uint8_t l3 : 1;
        uint8_t r3 : 1;
        uint8_t start : 1;

        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;

        uint8_t l2 : 1;
        uint8_t r2 : 1;
        uint8_t l1 : 1;
        uint8_t r1 : 1;

        uint8_t triangle : 1;
        uint8_t circle : 1;
        uint8_t cross : 1;
        uint8_t square : 1;

        uint8_t ps : 1;
    } ps3_button_t;

    /*******************************/
    /*   S T A T U S   F L A G S   */
    /*******************************/

    enum ps3_status_battery
    {
        ps3_status_battery_shutdown = 0x01,
        ps3_status_battery_dying = 0x02,
        ps3_status_battery_low = 0x03,
        ps3_status_battery_high = 0x04,
        ps3_status_battery_full = 0x05,
        ps3_status_battery_charging = 0xEE
    };

    enum ps3_status_connection
    {
        ps3_status_connection_usb,
        ps3_status_connection_bluetooth
    };

    typedef struct
    {
        enum ps3_status_battery battery;
        enum ps3_status_connection connection;
        uint8_t charging : 1;
        uint8_t rumbling : 1;
    } ps3_status_t;

    /********************/
    /*   S E N S O R S  */
    /********************/

    typedef struct
    {
        int16_t z;
    } ps3_sensor_gyroscope_t;

    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } ps3_sensor_accelerometer_t;

    typedef struct
    {
        ps3_sensor_accelerometer_t accelerometer;
        ps3_sensor_gyroscope_t gyroscope;
    } ps3_sensor_t;

    /*******************/
    /*    O T H E R    */
    /*******************/

    typedef struct
    {
        /* Rumble control */
        uint8_t rumble_right_duration;
        uint8_t rumble_right_intensity;
        uint8_t rumble_left_duration;
        uint8_t rumble_left_intensity;

        /* LED control */
        uint8_t led1 : 1;
        uint8_t led2 : 1;
        uint8_t led3 : 1;
        uint8_t led4 : 1;
    } ps3_cmd_t;

    typedef struct
    {
        ps3_button_t button_down;
        ps3_button_t button_up;
        ps3_analog_t analog_changed;
    } ps3_event_t;

    typedef struct
    {
        ps3_analog_t analog;
        ps3_button_t button;
        ps3_status_t status;
        ps3_sensor_t sensor;

    } ps3_t;
}

#endif