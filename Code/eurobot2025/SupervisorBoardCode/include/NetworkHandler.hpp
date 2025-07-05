#ifndef _NETWORK_HANDLER_H
#define _NETWORK_HANDLER_H


#include <IPAddress.h>

namespace NetworkHandler
{
    void init();
    IPAddress localIP();
}

#endif