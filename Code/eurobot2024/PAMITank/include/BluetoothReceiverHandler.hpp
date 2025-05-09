#ifndef BLUETOOTH_RECEIVER_HPP
#define BLUETOOTH_RECEIVER_HPP

#include <Ps3DataType.hpp>

namespace bluetooth_receiver_handler
{
    void init();
    bool newMessageReceived();
    const ps3_data_type::ps3_t* getData();
}

#endif