#include <Arduino.h>
#include <HeartbeatHandler.hpp>
#include <Strategy.hpp>
#include <Robot.hpp>
#include <LEDTimer.hpp>
#include <vector>

#include <FastLED.h>
#define FASTLED_PIN 0

// LED

#define LED_SLOW_BLINK_MS 1000
#define LED_FAST_BLINK_MS 150

#define LED_PIN 2
#define LED_PWM_CHANNEL 0

#define LED_PWM_FREQUENCY 1000
#define LED_PWM_RESOLUTION 8
#define LED_PWM_HIGH_LEVEL 32

// LED RING
#define LED_RING_BRIGHTNESS 6

// How many leds in your strip?
#define NUM_LEDS 16

namespace HeartbeatHandler
{
    CRGB leds[NUM_LEDS];

    void task_blink_led(void* parameters)
    {
        FastLED.setBrightness(LED_RING_BRIGHTNESS);
        FastLED.addLeds<WS2812, FASTLED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
        
        ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
        ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION);
        
        Robot* robot = Robot::getInstance();
        RobotState ms;

        // Back LED and status led
        PWMLEDTimer status_led_timer(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
        CRGBLEDTimer status_ring_timer(&(leds[0]));
        status_ring_timer.setColor(CRGB::Red);
        status_ring_timer.setPeriod(500);

        // Side indicator leds
        std::vector<std::shared_ptr<CRGBLEDTimer > > side_indicator_ring_timers;
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[1])));
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[2])));
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[3])));
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[13])));
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[14])));
        side_indicator_ring_timers.push_back(std::make_shared<CRGBLEDTimer >(&(leds[15])));

        for(;;)
        {
            // Handle status led 
            ms = robot->get_current_robot_state();

            // Led color
            if (ms == RobotState::MATCH_ENDED)
            {
                status_ring_timer.setColor(CRGB::Green);
            }
            else if (ms == RobotState::WAIT_FOR_CONFIGURATION || ms == RobotState::WAIT_FOR_MATCH_START)
            {
                status_ring_timer.setColor(CRGB::Purple);
            }
            else
            {
                status_ring_timer.setColor(CRGB::Red);
            }

            // Led period
            if (ms == RobotState::WAIT_FOR_CONFIGURATION)
            {
                // constant ON
                status_led_timer.setPeriod(0);
                status_ring_timer.setPeriod(0);
            }
            else if (ms == RobotState::MATCH_STARTED_WAITING ||
                ms == RobotState::MATCH_STARTED_ACTION ||
                ms == RobotState::MATCH_STARTED_FINAL_APPROACH ||
                ms == RobotState::MOVING_SETUP_TRAJECTORY)
            {
                // blink fast
                status_led_timer.setPeriod(LED_FAST_BLINK_MS);
                status_ring_timer.setPeriod(LED_FAST_BLINK_MS);
            }
            else
            {
                // blink slow
                status_led_timer.setPeriod(LED_SLOW_BLINK_MS);
                status_ring_timer.setPeriod(LED_SLOW_BLINK_MS);
            }
            status_led_timer.update();
            status_ring_timer.update();
            
            // Handle side indicator
            for (auto timer : side_indicator_ring_timers)
            {
                if (robot->motionController->isPlayingRightSide_)
                {
                    timer->setColor(CRGB::Yellow);
                }
                else
                {
                    timer->setColor(CRGB::Blue);
                }

                // Blink if last message was more than 3 sec ago
                if (millis() - robot->lastMessageReceivedTime_ > 3000)
                {
                    timer->setPeriod(300);
                }
                else
                {
                    timer->setPeriod(0);
                }

                timer->update();
            }

            FastLED.show();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }

    void start_heartbeat()
    {
        // start heartbeat
        Serial.println("Launch heartbeat");
        xTaskCreatePinnedToCore(
            task_blink_led, 
            "task_blink_led",
            10000,
            NULL,
            30,
            NULL,
            0 // pin to core 0
        );
    }
}