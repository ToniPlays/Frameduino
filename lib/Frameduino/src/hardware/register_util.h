#ifndef FRAMEDUINO_BOARDS_H
#define FRAMEDUINO_BOARDS_H

#include "hardware/gpio/hardware_pin.h"
#include "hardware/gpio/pin.h"

#include <stdint.h>

namespace Frameduino
{
    struct pin_info_t;
}

namespace Frameduino::HAL
{
    struct hardware_pin_t;
    struct hardware_pwm_t;

    volatile uint8_t *get_port_from_index(uint8_t port_index);
    volatile uint8_t *get_input_register_for_port(volatile uint8_t *port);
    volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port);

    const hardware_pin_t *get_hardware_pin_from_pin(uint8_t pin);
    const hardware_pwm_t *get_hardware_pwm_from_pin(pin_info_t *pin);

    bool pwm_write(pin_info_t *pin, uint16_t value);

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