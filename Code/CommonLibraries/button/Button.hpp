#ifndef _BUTTON_HPP
#define _BUTTON_HPP

#include <SoftwareButton.hpp>

class Button : public SoftwareButton
{
public:
    Button(char pin) : pin_(pin), SoftwareButton() { }
    Button(char pin, bool defaultState) : pin_(pin), SoftwareButton(defaultState) { }
    void init();
    void update();

private:
    char pin_;
};

#endif