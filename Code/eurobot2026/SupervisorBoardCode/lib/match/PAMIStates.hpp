#ifndef PAMI_STATES_HPP
#define PAMI_STATES_HPP

#include <Message.hpp>

namespace PAMIStates
{
    void registerMessage(std::shared_ptr<Message > message);
    PamiReportMessage readPAMIMessage(uint8_t pamiID);
    long readLastMessageTime();
}

#endif