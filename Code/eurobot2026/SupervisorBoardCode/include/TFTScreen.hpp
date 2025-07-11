#ifndef _TFT_SCREEN_H
#define _TFT_SCREEN_H

#include <Arduino.h>
#include <Message.hpp>

class TFTScreen
{
public:
    void init();
    void update(IPAddress localIP);
    void drawPAMI(FullPamiReportMessage pamiReport, uint8_t pamiID);
    void registerTouch();

private:
    IPAddress lastIP;
};

#endif