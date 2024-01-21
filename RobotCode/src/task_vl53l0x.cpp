#include <tasks.hpp>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t vlx_ranging_data_mm = 1000;

void init_vl53l0x(TwoWire* wire)
{
    // Serial.println("Init vlx");
    // Wire.begin(SDA, SCL, 400000);
    // sensor.setTimeout(500);
    // if (!sensor.init())
    // {
    //   Serial.println("Failed to detect and initialize sensor!");
    // }
    // else
    // {
    //     // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    //     // medium and long distance modes. See the VL53L1X datasheet for more
    //     // information on range and timing limits.
    //     sensor.setDistanceMode(VL53L1X::Long);
    //     sensor.setMeasurementTimingBudget(50000);

    //     // Start continuous readings at a rate of one measurement every 50 ms (the
    //     // inter-measurement period). This period should be at least as long as the
    //     // timing budget.
    //     sensor.startContinuous(50);
    // }

    // Wire.begin(SDA, SCL, 400000);
    if (!lox.begin(VL53L0X_I2C_ADDR, false,
                wire,
                Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT)) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    // // power
    // if (lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED))
    //   Serial.println("VL53L0X OK");
    // else
    //   Serial.println("Config failed");

    // start continuous ranging
    lox.startRangeContinuous(20);
}

uint16_t get_current_vl53l0x()
{
    return vlx_ranging_data_mm;
}

void update_vl53l0x()
{
    if (lox.isRangeComplete()) {
        vlx_ranging_data_mm = lox.readRange();
    }
}

// void task_update_vl53l0x(void* parameters)
// {
//     for (;;)
//     {
//         // get vlx data
//         // sensor.read();
//         // vlx_ranging_data_mm = sensor.ranging_data.range_mm;
//         if (lox.isRangeComplete()) {
//             vlx_ranging_data_mm = lox.readRange();
//         }
//         vTaskDelay(25 / portTICK_PERIOD_MS);
//     }
// }