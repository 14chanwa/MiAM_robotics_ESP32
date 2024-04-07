#ifndef MATCH_HPP
#define MATCH_HPP

#include <Message.hpp>

namespace Match
{
    void startMatch(float currentMatchTime = 0);
    void stopMatch();

    bool getMatchStarted();
    float getMatchTimeSeconds();

    void setSide(PlayingSide playingSide);
    PlayingSide getSide();
}

#endif