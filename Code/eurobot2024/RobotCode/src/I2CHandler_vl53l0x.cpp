#include <I2CHandler.hpp>

#include <Wire.h>

#define RIGHT_VLX_ENABLE 33
#define MIDDLE_VLX_ENABLE 32

#define LEFT_VLX_ADDRESS 0x40
#define MIDDLE_VLX_ADDRESS 0x35

namespace I2CHandler
{
    VLXSensor right_sensor;
    VLXSensor middle_sensor;
    VLXSensor left_sensor;

    void init_vl53l0x()
    {
        pinMode(RIGHT_VLX_ENABLE, OUTPUT);
        pinMode(MIDDLE_VLX_ENABLE, OUTPUT);

        digitalWrite(MIDDLE_VLX_ENABLE, LOW);
        digitalWrite(RIGHT_VLX_ENABLE, LOW);

        vTaskDelay(50 / portTICK_PERIOD_MS);
        left_sensor.init(LEFT_VLX_ADDRESS);

        digitalWrite(MIDDLE_VLX_ENABLE, HIGH);

        vTaskDelay(50 / portTICK_PERIOD_MS);
        middle_sensor.init(MIDDLE_VLX_ADDRESS);

        digitalWrite(RIGHT_VLX_ENABLE, HIGH);

        vTaskDelay(50 / portTICK_PERIOD_MS);
        right_sensor.init(VL53L0X_I2C_ADDR);

    };

    uint16_t get_current_vl53l0x()
    {
        return std::min(std::min(right_sensor.get_current(), middle_sensor.get_current()), left_sensor.get_current());
    };

    uint16_t get_smoothed_vl53l0x()
    {
        return std::min(std::min(right_sensor.get_smoothed(), middle_sensor.get_smoothed()), left_sensor.get_smoothed());
    };

    void update_vl53l0x()
    {
        right_sensor.update();
        middle_sensor.update();
        left_sensor.update();
    };

}

void VLXSensor::init(uint8_t target_i2c_addr)
{
    if (I2CHandler::i2c_get())
    {
        if (!lox_.begin(target_i2c_addr, true,
                        I2CHandler::get_wire(),
                        Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT))
        {
            Serial.println(F("Failed to boot VL53L0X"));
            while (1)
                ;
        }
        // start continuous ranging
        lox_.startRangeContinuous(20);

        // Release i2c
        I2CHandler::i2c_give();
    }
}

void VLXSensor::update()
{
    if (I2CHandler::i2c_get())
    {
        if (lox_.isRangeComplete())
        {
            current_ = lox_.readRange();
            smoothed_ = adc_.readADC_Avg(current_);
        }

        // Release i2c
        I2CHandler::i2c_give();
    }
}
