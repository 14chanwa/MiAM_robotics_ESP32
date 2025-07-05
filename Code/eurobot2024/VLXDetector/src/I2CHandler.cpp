#include <I2CHandler.hpp>
#include <Wire.h>
#include <WiFi.h>

SemaphoreHandle_t xMutex_I2C = NULL;
TwoWire* current_wire = nullptr;
TwoWire* secondary_wire = nullptr;

#define SDA2 16
#define SCL2 17

void task_update_vl53l0x(void* parameters)
{
    for (;;)
    {
      I2CHandler::update_vl53l0x();
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}
namespace I2CHandler
{
    void init()
    {
        Serial.println("Begin Wire");
        xMutex_I2C = xSemaphoreCreateMutex();  // crete a mutex object
        
        current_wire = new TwoWire(0);
        secondary_wire = new TwoWire(1);

        current_wire->begin(SDA, SCL, 400000);
        secondary_wire->begin(SDA2, SCL2, 400000);


        Serial.println("Init VL53L0X");
        I2CHandler::init_vl53l0x();

        xTaskCreatePinnedToCore(
            task_update_vl53l0x, 
            "task_update_vl53l0x",
            2000,
            NULL,
            10,  // mid priority
            NULL, 
            1 // pin to core 1
        ); 
    };

    TwoWire* get_wire()
    {
        return current_wire;
    };

    TwoWire* get_wire2()
    {
        return secondary_wire;
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