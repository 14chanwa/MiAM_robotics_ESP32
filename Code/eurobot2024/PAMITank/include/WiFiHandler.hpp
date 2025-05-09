#ifndef _WIFI_HANDLER_H
#define _WIFI_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <functional>

namespace wifi_handler
{
    void setBaseMACAddress(const uint8_t mac[6]);
    void printCurrentMACAddress();
    void setOTAOnStart(std::function<void(void)> fn);
    void connectSTA(String ssid, String password);
}

#endif