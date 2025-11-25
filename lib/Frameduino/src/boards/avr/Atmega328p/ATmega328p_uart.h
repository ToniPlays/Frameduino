#ifndef AVR_ATMEGA328P_UART
#define AVR_ATMEGA32P_UART

namespace Frameduino::HAL
{
    inline static uint32_t current_baud = 0;
    inline static uint8_t current_config = SERIAL_8N1;

    void uart_begin(uint32_t baud, uint8_t config = SERIAL_8N1)
    {
        current_baud = baud;
        current_config = config;

        uint32_t real_f_cpu = F_CPU >> (CLKPR & 0x0F);

        // Disable UART while reconfiguring
        UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0));

        // Try U2X first
        UCSR0A |= (1 << U2X0);

        // U2X formula with rounding
        uint16_t baud_setting =
            ((real_f_cpu + (baud * 2)) / (8UL * baud)) - 1;

        // Bootloader quirk + out-of-range check
        if ((real_f_cpu == 16000000UL && baud == 57600) || baud_setting > 4095)
        {
            // disable U2X
            UCSR0A &= ~(1 << U2X0);

            // normal mode formula with rounding
            baud_setting =
                ((real_f_cpu + (baud * 4)) / (16UL * baud)) - 1;
        }

        UBRR0H = baud_setting >> 8;
        UBRR0L = baud_setting & 0xFF;

        // set frame format
        UCSR0C = config;

        // enable UART
        UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
        UCSR0B &= ~(1 << UDRIE0);
    }

    bool uart_reset_baud_rate()
    {
        if (current_baud == 0)
            return false;

        uart_begin(current_baud, current_config);
        return true;
    }
}

#endif