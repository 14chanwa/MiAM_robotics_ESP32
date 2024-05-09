#include <I2CHandler.hpp>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#include <ADCReading.hpp>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
uint16_t vlx_ranging_data_mm = 1000;
uint16_t vlx_ranging_data_mm2 = 1000;

ADCReading vlx_ADC;
ADCReading vlx_ADC2;
uint16_t current_smoothed_vlx;
uint16_t current_smoothed_vlx2;

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

            if (!lox2.begin(VL53L0X_I2C_ADDR, false,
                I2CHandler::get_wire2(),
                Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT))
            {
                Serial.println(F("Failed to boot VL53L0X"));
                while (1)
                    ;
            }

            // start continuous ranging
            lox2.startRangeContinuous(20);

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

    uint16_t get_current_vl53l0x2()
    {
        return vlx_ranging_data_mm2;
    };

    uint16_t get_smoothed_vl53l0x2()
    {
        return current_smoothed_vlx2;
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

            if (lox2.isRangeComplete())
            {
                vlx_ranging_data_mm2 = lox2.readRange();
                current_smoothed_vlx2 = vlx_ADC2.readADC_Avg(vlx_ranging_data_mm2);
            }

            // Release i2c
            I2CHandler::i2c_give();
        }
    };

}
