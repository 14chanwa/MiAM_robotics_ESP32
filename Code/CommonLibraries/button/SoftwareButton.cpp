#include <SoftwareButton.hpp>
#include <Arduino.h>

#define DEBOUNCE_DELAY 200
#define DEFAULT_STATE HIGH

#define DEBUG_BUTTON
#ifdef DEBUG_BUTTON
    #ifdef CONFIG_IDF_TARGET
        #define DEBUG_PRINTLN(x) log_i(x)
    #else 
        #define DEBUG_PRINTLN(x) Serial.println(x)
    #endif
#else
    #define DEBUG_PRINTLN(x)
#endif

SoftwareButton::SoftwareButton() : SoftwareButton(DEFAULT_STATE)
{

}

SoftwareButton::SoftwareButton(bool defaultState) : 
    lastChange_(0), 
    lastEvent_(ButtonEvent::NO_EVENT),
    lastState_(defaultState)
{

}

void SoftwareButton::update(bool newState)
{
    // Do not update if state was not read
    if (lastEvent_ != ButtonEvent::NO_EVENT)
    {
        return;
    }

    // Update value
    currentState_ = newState;

    long currentTime = millis();
    if (currentTime - lastChange_ > DEBOUNCE_DELAY && 
        lastState_ != currentState_)
    {
        if (currentState_ == HIGH)
        {
            lastEvent_ = ButtonEvent::NEW_STATE_HIGH;
            DEBUG_PRINTLN("Trigger event HIGH");
        }
        else
        {
            lastEvent_ = ButtonEvent::NEW_STATE_LOW;
            DEBUG_PRINTLN("Trigger event LOW");
        }
        lastState_ = currentState_;
        lastChange_ = currentTime;
    }
}

ButtonEvent SoftwareButton::getEvent()
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

bool SoftwareButton::getValue()
{
    return lastState_;
}