#ifndef AVR_ATTINY1634_ENERGY_H
#define AVR_ATTINY1634_ENERGY_H

#include "system/core/energy.h"

namespace Frameduino::HAL
{
    bool set_clock_prescaler(uint8_t prescaler)
    {
        cli();
        // Apply new prescaler
        CLKPR = prescaler & 0x0F;

        for (uint8_t i = 0; i < 5; i++)
            asm volatile("nop");

        uart_reset_baud_rate();

        sei();

        return true;
    }

    void enable_sleep_mode(uint8_t mode)
    {
        switch (mode)
        {
        case FD_SLEEP_MODE_REDUCED:
        {
            MCUCR = BIT(SE);
            break;
        }
        case FD_SLEEP_MODE_POWER_DOWN:
        {
            MCUCR = BIT(SM0) | BIT(SE);
            break;
        }
        case FD_SLEEP_MODE_POWER_SAVE:
        {
            MCUCR = BIT(SM1) | BIT(SE);
            break;
        }
        case FD_SLEEP_MODE_STANDBY:
        {
            MCUCR = BIT(SM1) | BIT(SM0) | BIT(SE);
            break;
        }
        }
        asm volatile("sleep");
    }
}

#endif