#include <I2CHandler.hpp>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#include <ADCReading.hpp>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t vlx_ranging_data_mm = 1000;

ADCReading vlx_ADC;
uint16_t current_smoothed_vlx;

namespace I2CHandler
{
    void init_vl53l0x()
    {
        if (I2CHandler::i2c_get())
        {
            if (!lox.begin(VL53L0X_I2C_ADDR, false,
                           I2CHandler::get_wire(),
                           Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT))
            {
                Serial.println(F("Failed to boot VL53L0X"));
                while (1)
                    ;
            }

            // start continuous ranging
            lox.startRangeContinuous(20);

            // Release i2c
            I2CHandler::i2c_give();
        }
    };

    uint16_t get_current_vl53l0x()
    {
        return vlx_ranging_data_mm;
    };

    uint16_t get_smoothed_vl53l0x()
    {
        return current_smoothed_vlx;
    };

    void update_vl53l0x()
    {
        if (I2CHandler::i2c_get())
        {
            if (lox.isRangeComplete())
            {
                vlx_ranging_data_mm = lox.readRange();
                current_smoothed_vlx = vlx_ADC.readADC_Avg(vlx_ranging_data_mm);
            }

            // Release i2c
            I2CHandler::i2c_give();
        }
    };

}
