#include <Match.hpp>
#include <Arduino.h>


long matchTimeStarted = 0;
bool matchStarted = false;

void Match::startMatch()
{
    Serial.println(">>>>>>>>>>> Start match <<<<<<<<<<<");
    matchTimeStarted = millis();
    matchStarted = true;
}

void Match::stopMatch()
{
    Serial.println(">>>>>>>>>>> Stop match <<<<<<<<<<<");
    matchStarted = false;
}


bool Match::getMatchStarted()
{
    return matchStarted && Match::getMatchTimeSeconds() < 100.0f;
}

float Match::getMatchTimeSeconds()
{
    return (millis() - matchTimeStarted) / 1000.0f;
}