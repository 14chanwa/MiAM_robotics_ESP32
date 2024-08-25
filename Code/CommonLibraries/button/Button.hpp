#ifndef _BUTTON_HPP
#define _BUTTON_HPP

enum ButtonEvent
{
    NO_EVENT,
    NEW_STATE_HIGH,
    NEW_STATE_LOW
};

class Button
{
public:
    Button(char pin);
    Button(char pin, bool default_state);
    void init();
    void update();
    ButtonEvent getEvent(); 
    bool getValue();

private:
    char pin_;
    long lastChange_;
    bool lastPinState_;
    ButtonEvent lastEvent_;
};

#endif