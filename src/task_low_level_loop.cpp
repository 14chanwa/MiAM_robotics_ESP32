#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <PID.h>
#include <Motor.hpp>

void run_low_level_loop(Motor* motor)
{
    xTaskCreate(
        task_low_level_loop, 
        "task_low_level_loop",
        20000,
        motor,
        1,
        NULL
    ); 
    xTaskCreate(
        task_print_low_level, 
        "task_print_low_level",
        10000,
        motor,
        1,
        NULL
    ); 
}

void task_low_level_loop(void* parameters)
{
  for(;;)
  {
    Motor* obj = static_cast<Motor*>(parameters);
    obj->tickControl();
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
  }    
}


void task_print_low_level(void* parameters)
{
    for (;;)
    {
        Motor* obj = static_cast<Motor*>(parameters);  
        obj->printToSerial();     
        vTaskDelay(50.0 / portTICK_PERIOD_MS);
    }
}