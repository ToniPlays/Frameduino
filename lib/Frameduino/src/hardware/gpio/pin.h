#ifndef FRAMEDUINO_PIN_H
#define FRAMEDUINO_PIN_H

#include <stdint.h>
#include <Arduino.h>

#include "operators.h"
#include "hardware_pin.h"

namespace Frameduino
{
    enum pin_type : uint8_t
    {
        PIN_CONFIG_DIGITAL_INPUT = BIT(0),
        PIN_CONFIG_DIGITAL_OUTPUT = BIT(1),
        PIN_CONFIG_ANALOG_INPUT = BIT(2),
        PIN_CONFIG_ANALOG_OUTPUT = BIT(3),
        PIN_CONFIG_PWM = BIT(4),
    };

    enum pin_interrupt : uint8_t
    {
        PIN_INTERRUPT_RISING = BIT(0),
        PIN_INTERRUPT_FALLING = BIT(1),
        PIN_INTERRUPT_CHANGE = PIN_INTERRUPT_RISING | PIN_INTERRUPT_FALLING
    };

    struct pin_info_t
    {
        uint8_t pin_number;
        volatile uint8_t* port;
        uint8_t mask; // Port pin mask
    };

    bool hal_pin_attach(uint8_t pin, uint8_t usage, pin_info_t *info);
    bool hal_pin_detach(pin_info_t info);

    inline bool hal_pin_attach_interrupt(pin_info_t* pin, uint8_t mask, void(* cb)(void*), void* data = nullptr) { return false; }
    inline bool hal_pin_detach_interrupt(pin_info_t* pin, uint8_t mask) { return false; }

    inline void hal_pin_write(pin_info_t* info, bool value)
    {
        HAL::port_bit_write(info->port, info->mask, value);
    }

    inline void hal_pin_toggle(pin_info_t* info)
    {
        *info->port ^= info->mask;
    }
    inline void hal_pin_pulse(pin_info_t info, uint32_t duration, bool state = true)
    {
        HAL::port_bit_write(info.port, info.mask, state);
        delay(duration);
        HAL::port_bit_write(info.port, info.mask, !state);
    }

    inline bool hal_pwm_write(pin_info_t* info, uint16_t value)
    {
        return HAL::pwm_write(info, value);
    }

    inline bool pin_supports_usage(uint8_t caps, uint8_t usage)
    {
        using namespace HAL;

        if (usage & PIN_CONFIG_ANALOG_INPUT)
            return (caps & (PIN_CAP_INPUT | PIN_CAP_ADC)) != 0;
        if (usage & PIN_CONFIG_ANALOG_OUTPUT)
            return (caps & (PIN_CAP_DAC | PIN_CAP_OUTPUT)) != 0;
        if (usage & PIN_CONFIG_DIGITAL_INPUT)
            return (caps & PIN_CAP_INPUT) != 0;
        if (usage & PIN_CONFIG_DIGITAL_OUTPUT)
            return (caps & PIN_CAP_OUTPUT) != 0;
        return 0;
    }
}

#endif