#ifndef _TFT_SCREEN_H
#define _TFT_SCREEN_H

#include <Arduino.h>
#include <Message.hpp>

class TFTScreen
{
public:
    void init();
    void update(IPAddress localIP);
    void drawPAMI(PamiReportMessage pamiReport, uint8_t pamiID);

    static void registerMessage(std::shared_ptr<Message > message);
    static PamiReportMessage readPAMIMessage(uint8_t pamiID);
    static long readLastMessageTime();
};

#endif