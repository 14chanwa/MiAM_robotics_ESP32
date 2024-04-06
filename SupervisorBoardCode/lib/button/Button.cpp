#include <Button.hpp>
#include <Arduino.h>

#define DEBOUNCE_DELAY 200
#define DEFAULT_STATE HIGH

#define DEBUG_BUTTON

Button::Button(char pin) : 
    pin_(pin), 
    lastChange_(0), 
    lastEvent_(ButtonEvent::NO_EVENT),
    lastPinState_(DEFAULT_STATE)
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
#ifdef DEBUG_BUTTON
            Serial.println("Trigger event HIGH");
#endif
        }
        else
        {
#ifdef DEBUG_BUTTON
            lastEvent_ = ButtonEvent::NEW_STATE_LOW;
            Serial.println("Trigger event LOW");
#endif
        }
        lastPinState_ = currentPinState;
        lastChange_ = currentTime;
    }
}

ButtonEvent Button::getEvent()
{
    ButtonEvent readEvent = lastEvent_;
    if (readEvent == ButtonEvent::NEW_STATE_HIGH)
#ifdef DEBUG_BUTTON
        Serial.println("getEvent: HIGH");
    else if (readEvent == ButtonEvent::NEW_STATE_LOW)
        Serial.println("getEvent: LOW");
#endif
    lastEvent_ = ButtonEvent::NO_EVENT;
    return readEvent;
}
