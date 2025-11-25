#ifndef FRAMEDUINO_ENERGY_H
#define FRAMEDUINO_ENERGY_H

#include "boards/hal.h"
#include "hal_system.h"

#define SLEEP_MODE_REDUCED 0x00
#define SLEEP_MODE_POWER_DOWN 0x01
#define SLEEP_MODE_POWER_SAVE 0x02
#define SLEEP_MODE_STANDBY 0x03

namespace Frameduino
{
    inline void hal_set_clock_prescaler(uint8_t prescaler)
    {
        hal_logger_log_w(("Setting clock prescaler: " + String(prescaler)).c_str());
        HAL::set_clock_prescaler(prescaler);
        
    }

    inline void hal_system_sleep(uint8_t mode)
    {
        hal_logger_log_w(("Settings sleep mode: " + String(mode)).c_str());
        HAL::enable_sleep_mode(mode);
    }
}

#endif