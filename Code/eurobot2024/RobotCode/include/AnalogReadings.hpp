#ifndef _ANALOG_READINGS_H
#define _ANALOG_READINGS_H

namespace AnalogReadings
{
    void init();
    void update();

    float get_current_battery_reading();
    float get_min_battery_reading();
    // float get_current_tcrt0_reading();
    // float get_current_tcrt1_reading();
    // float get_current_tcrt2_reading();
    // float get_current_touch_sensor_reading();

    bool get_left_switch_value();
    bool get_right_switch_value();
}

#endif