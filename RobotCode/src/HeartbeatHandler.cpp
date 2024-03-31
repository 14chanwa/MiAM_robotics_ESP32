#include <Arduino.h>
#include <HeartbeatHandler.hpp>
#include <Strategy.hpp>
#include <Robot.hpp>

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

    RobotState ms;
    Robot* robot = Robot::getInstance();

    for (;;)
    {
        ms = robot->get_current_robot_state();
        if (ms == RobotState::WAIT_FOR_CONFIGURATION)
        {
            // constant ON
            ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
            vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
        }
        else if (ms == RobotState::MATCH_STARTED_WAITING ||
            ms == RobotState::MATCH_STARTED_ACTION ||
            ms == RobotState::MATCH_STARTED_FINAL_APPROACH)
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