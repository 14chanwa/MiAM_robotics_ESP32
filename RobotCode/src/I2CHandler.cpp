#include <I2CHandler.hpp>
#include <Wire.h>

SemaphoreHandle_t xMutex_I2C = NULL;
TwoWire* current_wire = nullptr;

namespace I2CHandler
{
    void init()
    {
        Serial.println("Begin Wire");
        xMutex_I2C = xSemaphoreCreateMutex();  // crete a mutex object
        Wire.begin(SDA, SCL, 400000);
        current_wire = &Wire;
        Serial.println("Init VL53L0X");
        I2CHandler::init_vl53l0x();
        Serial.println("Init SSD1306");
        I2CHandler::initOLEDScreen();
    };

    TwoWire* get_wire()
    {
        return current_wire;
    };

    bool i2c_get()
    {
        return xSemaphoreTake(xMutex_I2C, portMAX_DELAY);
    };

    bool i2c_give()
    {
        return xSemaphoreGive(xMutex_I2C);  // release the mutex
    };
}