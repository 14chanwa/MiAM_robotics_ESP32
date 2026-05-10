#include <AnalogReadings.hpp>
#include <Arduino.h>
#include <parameters.hpp>
#include <ADCReading.hpp>

/////////////////////////////////////////////
// Resistor values
/////////////////////////////////////////////

#define RESISTOR_R1 100000.0f
#define RESISTOR_R2 10000.0f

#define BAT_READING 36

// #define TCRT_0 32
// #define TCRT_1 39
// #define TCRT_2 33

// #define TOUCH_SENSOR 0

// #define LEFT_SWITCH_PIN 33
#define RIGHT_SWITCH_PIN 39

/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

ADCReading battery_reading;
ADCReading tcrt0_reading;
ADCReading tcrt1_reading;
ADCReading tcrt2_reading;

ADCReading touch_sensor_reading;

// float batReading_unfiltered = 0;
float batReading = 0;
float minBatReading = 100;

// float tcrt0Value = 0;
// float tcrt1Value = 0;
// float tcrt2Value = 0;
// float touchSensorValue = 0;

bool leftSwitchValue_;
bool rightSwitchValue_;

// void print_battery()
// {
//   Serial.print(">batReading:");
//   Serial.println(batReading);
//   Serial.print(">minBatReading:");
//   Serial.println(minBatReading);
// }

// void task_update_analog_readings(void* parameters){
//     for(;;)
//     {
//       monitor_battery();
//       vTaskDelay(20 / portTICK_PERIOD_MS);
//     }
// }

namespace AnalogReadings
{
    void init()
    {
        pinMode(BAT_READING, INPUT_PULLDOWN);
        // pinMode(TCRT_0, INPUT_PULLDOWN);
        // pinMode(TCRT_1, INPUT_PULLDOWN);
        // pinMode(TCRT_2, INPUT_PULLDOWN);
        analogReadResolution(12);
        analogSetAttenuation(ADC_11db);

        // pinMode(LEFT_SWITCH_PIN, INPUT_PULLDOWN);
        pinMode(RIGHT_SWITCH_PIN, INPUT_PULLDOWN);
    }

    void update()
    {
        int AN_Pot1_Raw = analogReadMilliVolts(BAT_READING);
        // batReading_unfiltered = AN_Pot1_Raw * (RESISTOR_R1 + RESISTOR_R2) / R2 / 1000.0;
        batReading = battery_reading.readADC_Avg(AN_Pot1_Raw) * (RESISTOR_R1 + RESISTOR_R2) / RESISTOR_R2 / 1000.0;
        // batReading = battery_reading.readADC_Avg(AN_Pot1_Raw) / 1000.0;
        // tcrt0Value = tcrt0_reading.readADC_Avg(analogReadMilliVolts(TCRT_0)) / 1000.0;
        // tcrt1Value = tcrt1_reading.readADC_Avg(analogReadMilliVolts(TCRT_1)) / 1000.0;
        // tcrt2Value = tcrt2_reading.readADC_Avg(analogReadMilliVolts(TCRT_2)) / 1000.0;
        // touchSensorValue = touch_sensor_reading.readADC_Avg(touchRead(TOUCH_SENSOR)) / 1000.0;
        minBatReading = std::min(minBatReading, batReading);

        leftSwitchValue_ = digitalRead(RIGHT_SWITCH_PIN);
        rightSwitchValue_ = digitalRead(RIGHT_SWITCH_PIN);
    }

    float get_current_battery_reading()
    {
        return batReading;
    }

    float get_min_battery_reading()
    {
        return minBatReading;
    }

    // float get_current_tcrt0_reading()
    // {
    //     return tcrt0Value;
    // }

    // float get_current_tcrt1_reading()
    // {
    //     return tcrt1Value;
    // }

    // float get_current_tcrt2_reading()
    // {
    //     return tcrt2Value;
    // }

    // float get_current_touch_sensor_reading()
    // {
    //     return touchSensorValue;
    // }


    bool get_left_switch_value()
    {
        return leftSwitchValue_;
    }

    bool get_right_switch_value()
    {
        return rightSwitchValue_;
    }
}
