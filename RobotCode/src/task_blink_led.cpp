#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>

void task_blink_led(void* parameters)
{
  for(;;)
  {
    ledcWrite(0, 32);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ledcWrite(0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}