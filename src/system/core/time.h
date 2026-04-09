#ifndef FRAMEDUINO_CORE_TIME_H
#define FRAMEDUINO_CORE_TIME_H

#include "system/core/hal_system.h"
#include <Wire.h>

namespace Frameduino
{
    inline static void hal_time_update(uint32_t micros)
    {
        micros = micros << hal_clock_prescaler();
        
        system_info->micros += micros;
        system_info->millis = system_info->micros / 1000.0f;
    }

    inline static uint64_t hal_millis()
    {
        return system_info->millis;
    }

    inline static uint64_t hal_micros()
    {
        return system_info->micros;
    }
}

#endif