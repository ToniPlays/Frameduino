#ifndef FRAMEDUINO_BOARDS_H
#define FRAMEDUINO_BOARDS_H

#include "avr/Boards.h"
#include "hardware/gpio/HardwarePin.h"

namespace Frameduino::HAL
{
    static volatile uint8_t *get_port_from_index(uint8_t port_index);
    static volatile uint8_t *get_input_register_for_port(volatile uint8_t *port);
    static volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port);
    static HardwarePin get_hardware_pin_from_pin(uint8_t pin);

    inline void port_bit_write(volatile uint8_t *target, uint8_t mask, bool value)
    {
        *target = (*target & ~mask) | (value ? mask : 0);
    }

    inline bool port_bit_read(volatile uint8_t *reg, uint8_t mask)
    {
        return (*reg & mask) != 0;
    }

    inline bool port_bit_read_input(volatile uint8_t *target, uint8_t mask)
    {
        volatile uint8_t *pin_reg = get_input_register_for_port(target);
        return (*pin_reg & mask) != 0; // read physical voltage
    }
}
#endif