#include <I2CHandler.hpp>
#include <Wire.h>
#include <WiFi.h>
#include <Robot.hpp>

SemaphoreHandle_t xMutex_I2C = NULL;
TwoWire* current_wire = nullptr;

// Display
DisplayInformations display_informations;

void task_update_vl53l0x(void* parameters)
{
    for (;;)
    {
      I2CHandler::update_vl53l0x();
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

void task_update_ssd1306(void* parameters)
{
    IPAddress ip;
    Robot* robot = Robot::getInstance();
    for (;;)
    {
        // Update IP
        ip = WiFi.localIP();
        sprintf(display_informations.ip_address,"%d:%d:%d:%d", ip[0],ip[1],ip[2],ip[3]);

        // Update ID
        display_informations.id = robot->robotID;

        // Update match state
        display_informations.match_started = robot->matchStarted();
        display_informations.current_time_s = std::round(robot->match_current_time_s);

        // Update display
        I2CHandler::update_ssd1306(&display_informations);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


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

        xTaskCreatePinnedToCore(
            task_update_vl53l0x, 
            "task_update_vl53l0x",
            2000,
            NULL,
            10,  // mid priority
            NULL, 
            1 // pin to core 1
        ); 

        xTaskCreatePinnedToCore(
            task_update_ssd1306, 
            "task_update_ssd1306",
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

    bool i2c_get()
    {
        return xSemaphoreTake(xMutex_I2C, portMAX_DELAY);
    };

    bool i2c_give()
    {
        return xSemaphoreGive(xMutex_I2C);  // release the mutex
    };
}