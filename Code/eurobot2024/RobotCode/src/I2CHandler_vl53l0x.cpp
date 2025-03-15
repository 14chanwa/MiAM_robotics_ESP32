#include <I2CHandler.hpp>

#include <Wire.h>

#define RIGHT_VLX_ENABLE 33
#define MIDDLE_VLX_ENABLE 32
#define LEFT_VLX_ENABLE 5

#define LEFT_VLX_ADDRESS 0x31
#define MIDDLE_VLX_ADDRESS 0x30

namespace I2CHandler
{

    VLXSensor right_sensor;
    VLXSensor middle_sensor;
    VLXSensor left_sensor;

    void init_vl53l0x()
    {
        pinMode(RIGHT_VLX_ENABLE, OUTPUT);
        pinMode(MIDDLE_VLX_ENABLE, OUTPUT);
        pinMode(LEFT_VLX_ENABLE, OUTPUT);

        // Reset all vlx : low, high, low
        digitalWrite(MIDDLE_VLX_ENABLE, LOW);
        digitalWrite(RIGHT_VLX_ENABLE, LOW);
        digitalWrite(LEFT_VLX_ENABLE, LOW);

        vTaskDelay(10 / portTICK_PERIOD_MS);

        digitalWrite(MIDDLE_VLX_ENABLE, HIGH);
        digitalWrite(RIGHT_VLX_ENABLE, HIGH);
        digitalWrite(LEFT_VLX_ENABLE, HIGH);

        vTaskDelay(10 / portTICK_PERIOD_MS);

        digitalWrite(MIDDLE_VLX_ENABLE, LOW);
        digitalWrite(RIGHT_VLX_ENABLE, LOW);
        digitalWrite(LEFT_VLX_ENABLE, LOW);


        // Init left vlx
        vTaskDelay(50 / portTICK_PERIOD_MS);
        digitalWrite(LEFT_VLX_ENABLE, HIGH);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        left_sensor.init(LEFT_VLX_ADDRESS);

        // Init middle vlx
        vTaskDelay(50 / portTICK_PERIOD_MS);
        digitalWrite(MIDDLE_VLX_ENABLE, HIGH);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        middle_sensor.init(MIDDLE_VLX_ADDRESS);

        // Init right vlx
        vTaskDelay(50 / portTICK_PERIOD_MS);
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

    int16_t get_smoothed_vlx_side(Side side)
    {
        switch (side)
        {
            case RIGHT:
                if (right_sensor.is_init())
                {
                    return right_sensor.get_smoothed();
                }
                else
                {
                    return -1;
                }
                break;
            case MIDDLE:
                if (middle_sensor.is_init())
                {
                    return middle_sensor.get_smoothed();
                }
                else
                {
                    return -1;
                }
                break;
            case LEFT:
                if (left_sensor.is_init())
                {
                    return left_sensor.get_smoothed();
                }
                else
                {
                    return -1;
                }
                break;
            default:
                return -1;
        }
    }


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
        if (lox_.begin(target_i2c_addr, true,
                        I2CHandler::get_wire(),
                        Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT))
        {
            // start continuous ranging
            lox_.startRangeContinuous(20);
            is_init_ = true;
        }
        else
        {
            Serial.println(F("Failed to boot VL53L0X"));
        }

        // Release i2c
        I2CHandler::i2c_give();
    }
}

void VLXSensor::update()
{
    if (I2CHandler::i2c_get())
    {
        if (is_init_ && lox_.isRangeComplete())
        {
            current_ = lox_.readRange();
            smoothed_ = adc_.readADC_Avg(current_);
        }

        // Release i2c
        I2CHandler::i2c_give();
    }
}
