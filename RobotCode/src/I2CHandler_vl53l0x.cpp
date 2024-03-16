#include <I2CHandler.hpp>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t vlx_ranging_data_mm = 1000;

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

    void update_vl53l0x()
    {
        if (I2CHandler::i2c_get())
        {
            if (lox.isRangeComplete())
            {
                vlx_ranging_data_mm = lox.readRange();
            }

            // Release i2c
            I2CHandler::i2c_give();
        }
    };

}
