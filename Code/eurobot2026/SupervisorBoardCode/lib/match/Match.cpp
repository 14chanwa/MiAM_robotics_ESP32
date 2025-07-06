#include <Match.hpp>
#include <Arduino.h>


long matchTimeStarted = 0;
bool matchStarted = false;
PlayingSide currentPlayingSide = PlayingSide::BLUE_SIDE;
bool stopMotors = false;

void Match::startMatch(float currentMatchTime)
{
    Serial.println(">>>>>>>>>>> Start match <<<<<<<<<<<");
    matchTimeStarted = millis() - (long)(currentMatchTime*1000);
    matchStarted = true;
}

void Match::stopMatch()
{
    Serial.println(">>>>>>>>>>> Stop match <<<<<<<<<<<");
    matchStarted = false;
}


bool Match::getMatchStarted()
{
    return matchStarted;
}

float Match::getMatchTimeSeconds()
{
    if (!getMatchStarted())
    {
        return 0.0;
    }
    return std::min((millis() - matchTimeStarted) / 1000.0f, 100.0f);
}

void Match::setSide(PlayingSide playingSide)
{
    currentPlayingSide = playingSide;
}

PlayingSide Match::getSide()
{
    return currentPlayingSide;
}

bool Match::getStopMotors()
{
    return stopMotors;
}

void Match::setStopMotors(bool value)
{
    stopMotors = value;
}