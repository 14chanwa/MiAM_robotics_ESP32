#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>

void run_blink_led()
{
    pinMode(LED_BUILTIN, OUTPUT);
    xTaskCreate(
        task_blink_led, 
        "task_blink_led",
        1000,
        NULL,
        1,
        NULL
    ); 
}

void task_blink_led(void* parameters)
{
  for(;;)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(1000.0 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(1000.0 / portTICK_PERIOD_MS);
  }
}