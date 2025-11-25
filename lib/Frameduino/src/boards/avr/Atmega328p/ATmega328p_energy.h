#ifndef AVR_ATMEGA328P_UART
#define AVR_ATMEGA32P_UART

#include "system/core/energy.h"

namespace Frameduino::HAL
{
    bool set_clock_prescaler(uint8_t prescaler)
    {
        cli();
        // Apply new prescaler
        CLKPR = (1 << CLKPCE);
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
        case SLEEP_MODE_REDUCED:
        {
            SMCR = BIT(SM0) | BIT(SE);
            break;
        }
        case SLEEP_MODE_POWER_DOWN:
        {
            SMCR = BIT(SM1) | BIT(SE);
            break;
        }
        case SLEEP_MODE_POWER_SAVE:
        {
            SMCR = BIT(SM1) | BIT(SM0) | BIT(SE);
            break;
        }
        case SLEEP_MODE_STANDBY:
        {
            SMCR = BIT(SM2) | BIT(SM1) | BIT(SE);
            break;
        }
        }
        asm volatile("sleep");
    }
}

#endif