#ifndef _ABSTRACT_BUTTON_H
#define _ABSTRACT_BUTTON_H

enum ButtonEvent
{
    NO_EVENT,
    NEW_STATE_HIGH,
    NEW_STATE_LOW
};

class SoftwareButton
{
public:
    SoftwareButton();
    SoftwareButton(bool defaultState);

    void update(bool newValue);

    ButtonEvent getEvent(); 
    bool getValue();

protected:
    long lastChange_;
    bool lastState_;
    bool currentState_;
    ButtonEvent lastEvent_;
};

#endif