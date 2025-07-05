#ifndef _TFT_SCREEN_H
#define _TFT_SCREEN_H

#include <Arduino.h>

class TFTScreen
{
public:
    void init();
    void update();
    void registerTouch();
};

#endif