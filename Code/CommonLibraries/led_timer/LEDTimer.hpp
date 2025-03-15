#ifndef _LED_TIMER_H
#define _LED_TIMER_H

#include <Arduino.h>
#include <FastLED.h>

class LEDTimer
{
    public:
        LEDTimer()  :
            timer_ms_(0),
            period_ms_(0),
            current_state_(false)
        {}

        void setPeriod(long period_ms) { period_ms_ = period_ms; }
        bool get_new_led_state();
        virtual void update() = 0;
    
    private:
        long timer_ms_;
        long period_ms_;
        bool current_state_;
};

class CRGBLEDTimer : public LEDTimer
{
    public:
        CRGBLEDTimer(CRGB* led) : 
            LEDTimer(), 
            color_(CRGB::Black),
            led_(led),
            color_changed_(false)
        {}

        void setColor(CRGB color) { 
            if (color_ != color)
            {
                color_ = color;
                color_changed_ = true;
            }
        }
        void update();

    private:
        CRGB* led_;
        CRGB color_;
        bool color_changed_;
};

class PWMLEDTimer : public LEDTimer
{
    public:
        PWMLEDTimer(uint8_t channel, uint32_t duty) :
            LEDTimer(),
            channel_(channel),
            duty_(duty)
        {}
        
        void update();
    
    private:
        uint8_t channel_;
        uint32_t duty_;
};

void CRGBLEDTimer::update()
{
    if (get_new_led_state() || color_changed_)
    {
        *led_ = color_;
        color_changed_ = false;
    }
    else
    {
        *led_ = CRGB(0,0,0);
    }
}

void PWMLEDTimer::update()
{
    if (get_new_led_state())
    {
        ledcWrite(channel_, duty_);
    }
    else
    {
        ledcWrite(channel_, 0);
    }
}

bool LEDTimer::get_new_led_state()
{
    long new_time = millis();
    // If period > 0, update timer
    if (period_ms_ > 0)
    {
        // Change state if period is elapsed
        if (new_time - timer_ms_ > period_ms_)
        {
            current_state_ = !current_state_;
            timer_ms_ = new_time;
        }
        return current_state_;
    }
    // If period == 0 then display solid color
    else
    {
        timer_ms_ = new_time;
        return true;
    }
}

#endif