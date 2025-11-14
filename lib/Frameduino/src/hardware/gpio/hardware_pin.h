#ifndef FRAMEDUINO_HARDWARE_PIN_H
#define FRAMEDUINO_HARDWARE_PIN_H

#include <stdint.h>
#include "operators.h"

namespace Frameduino {
    struct pin_info_t;
}

namespace Frameduino::HAL
{

    enum pin_capability_t : uint16_t
    {
        PIN_CAP_OUTPUT = BIT(0),
        PIN_CAP_INPUT = BIT(1),
        PIN_CAP_PWM = BIT(2),
        PIN_CAP_RESET = BIT(3),
        PIN_CAP_AREF = BIT(4),
        PIN_CAP_TIMER = BIT(5),
        PIN_CAP_UART = BIT(6),
        PIN_CAP_SPI = BIT(7),
        PIN_CAP_I2C = BIT(8),
        PIN_CAP_ADC = BIT(9),
        PIN_CAP_DAC = BIT(10),
        PIN_CAP_INT = BIT(11),
        PIN_CAP_PCINT = BIT(11),

        PIN_CAP_IO = PIN_CAP_INPUT | PIN_CAP_OUTPUT,
        PIN_CAP_INTERRUPTS = PIN_CAP_INT | PIN_CAP_PCINT,
    };

    struct hardware_pin_t
    {
        uint8_t pin_number;
        uint8_t port;
        uint8_t pin;
        uint16_t capabilities;

        constexpr hardware_pin_t() : pin_number(0), port(0), pin(0), capabilities(0) {};
        constexpr hardware_pin_t(uint16_t number, uint8_t port, uint8_t pin, uint32_t caps) : pin_number(number), port(port), pin(pin), capabilities(caps) {}
    };

    struct hardware_pwm_t
    {
        uint8_t pin_number;
        uint8_t timer;
        int8_t channel;
        uint8_t scalar;

        constexpr hardware_pwm_t() : pin_number(0), timer(0), channel(-1), scalar(0) {};
        constexpr hardware_pwm_t(uint16_t number, uint8_t timer, uint8_t channel, uint8_t scalar) : pin_number(number), timer(timer), channel(channel), scalar(scalar) {}
    };

    

    volatile uint8_t *get_port_from_index(uint8_t port_index);
    volatile uint8_t *get_input_register_for_port(volatile uint8_t *port);
    volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port);

    const hardware_pin_t get_hardware_pin_from_pin(uint8_t pin);
    const hardware_pwm_t get_hardware_pwm_from_pin(pin_info_t *pin);

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