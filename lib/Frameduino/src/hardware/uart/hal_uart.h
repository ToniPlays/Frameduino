#ifndef FRAMEDUINO_UART_H
#define FRAMEDUINO_UART_H

#include "boards/hal.h"

namespace Frameduino
{
    inline void hal_uart_enable(uint32_t baud)
    {
        HAL::uart_begin(baud, 0x06);
    }
}

#endif