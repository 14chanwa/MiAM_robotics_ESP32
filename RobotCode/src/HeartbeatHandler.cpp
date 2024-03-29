#include <Arduino.h>
#include <HeartbeatHandler.hpp>
#include <Strategy.hpp>

#define LED_SLOW_BLINK_MS 1000
#define LED_FAST_BLINK_MS 150

#define LED_PIN 2
#define LED_PWM_CHANNEL 0

#define LED_PWM_FREQUENCY 1000
#define LED_PWM_RESOLUTION 8
#define LED_PWM_HIGH_LEVEL 32

using namespace strategy;

/// @brief Tasks which blinks led according to match state
/// @param parameters 
void task_blink_led(void *parameters)
{
    ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION);
    ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);

    MatchState ms;
    for (;;)
    {
        ms = get_current_match_state();
        if (ms == MatchState::WAIT_FOR_CONFIGURATION)
        {
            // constant ON
            ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
        }
        else if (ms == MatchState::MATCH_STARTED_WAITING ||
            ms == MatchState::MATCH_STARTED_ACTION ||
            ms == MatchState::MATCH_STARTED_FINAL_APPROACH)
        {
            // blink fast
            ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
            vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
            ledcWrite(LED_PWM_CHANNEL, 0);
            vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
        }
        else
        {
            // blink slow
            ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
            vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
            ledcWrite(LED_PWM_CHANNEL, 0);
            vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
        }
    }
}

namespace HeartbeatHandler
{

    void start_heartbeat()
    {
        // start heartbeat
        Serial.println("Launch heartbeat");
        xTaskCreatePinnedToCore(
            task_blink_led, 
            "task_blink_led",
            1000,
            NULL,
            1,
            NULL,
            0 // pin to core 0
        ); 
    }

}