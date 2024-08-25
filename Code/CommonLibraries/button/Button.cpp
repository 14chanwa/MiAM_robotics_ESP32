#include <Button.hpp>
#include <Arduino.h>

#define DEBOUNCE_DELAY 200
#define DEFAULT_STATE HIGH

#define DEBUG_BUTTON
#ifdef DEBUG_BUTTON
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

Button::Button(char pin) : Button(pin, DEFAULT_STATE)
{

}

Button::Button(char pin, bool default_state) : 
    pin_(pin), 
    lastChange_(0), 
    lastEvent_(ButtonEvent::NO_EVENT),
    lastPinState_(default_state)
{

}

void Button::init()
{
    pinMode(pin_, INPUT);
}

void Button::update()
{
    // Do not update if state was not read
    if (lastEvent_ != ButtonEvent::NO_EVENT)
    {
        return;
    }

    // Else monitor changes
    long currentTime = millis();
    bool currentPinState = digitalRead(pin_);
    if (currentTime - lastChange_ > DEBOUNCE_DELAY && 
        lastPinState_ != currentPinState)
    {
        if (currentPinState == HIGH)
        {
            lastEvent_ = ButtonEvent::NEW_STATE_HIGH;
            DEBUG_PRINTLN("Trigger event HIGH");
        }
        else
        {
            lastEvent_ = ButtonEvent::NEW_STATE_LOW;
            DEBUG_PRINTLN("Trigger event LOW");
        }
        lastPinState_ = currentPinState;
        lastChange_ = currentTime;
    }
}

ButtonEvent Button::getEvent()
{
    ButtonEvent readEvent = lastEvent_;
    if (readEvent == ButtonEvent::NEW_STATE_HIGH)
    {
        DEBUG_PRINTLN("getEvent: HIGH");
    }
    else if (readEvent == ButtonEvent::NEW_STATE_LOW)
    {
        DEBUG_PRINTLN("getEvent: LOW");
    }
    lastEvent_ = ButtonEvent::NO_EVENT;
    return readEvent;
}

bool Button::getValue()
{
    return lastPinState_;
}