#include <Arduino.h>
#include <pinout.hpp>
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
    // Serial.print("task_blink_led is running on: ");
    // Serial.println(xPortGetCoreID());
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(1000.0 / portTICK_PERIOD_MS);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(1000.0 / portTICK_PERIOD_MS);
  }
}