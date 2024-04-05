#ifndef _WIFI_HANDLER_H
#define _WIFI_HANDLER_H

#include <WiFi.h>

namespace WiFiHandler
{
    void initWiFi();
    bool sendTCPMessage(char* message, uint message_size, IPAddress ip_addr, uint port);
    void startReportBroadcast();
}

#endif