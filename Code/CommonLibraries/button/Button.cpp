#include <Button.hpp>
#include <Arduino.h>

void Button::init()
{
    pinMode(pin_, INPUT);
}

void Button::update()
{
    SoftwareButton::update(digitalRead(pin_));
}

