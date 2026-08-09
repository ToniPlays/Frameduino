#ifndef FRAMEDUINO_CORE_TIME_H
#define FRAMEDUINO_CORE_TIME_H

#include "system/core/hal_system.h"
#include <Wire.h>

namespace Frameduino
{
    inline static void hal_time_update(uint32_t micros)
    {
        micros = micros << hal_clock_prescaler();
        static uint16_t micros_count = 0;
        micros_count += micros;

        system_info->micros += micros;
        if(micros_count < 1000) return;

        system_info->millis += micros_count / 1000;
        micros_count = micros_count % 1000;
    }

    inline static uint64_t hal_millis()
    {
        return system_info->millis;
    }

    inline static uint64_t hal_micros()
    {
        return system_info->micros;
    }

    inline static void hal_delay_millis(uint32_t ms)
    {
        uint64_t start = hal_millis();
        while (hal_millis() - start < ms) {
            asm volatile ("nop");
        };
    }
    inline static void hal_delay_micros(uint32_t us)
    {
        uint64_t start = hal_micros();
        while (hal_micros() - start < us) {
            asm volatile ("nop");
        };
    }
}

#endif