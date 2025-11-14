#ifndef FRAMEDUINO_HAL_SYSTEM_H
#define FRAMEDUINO_HAL_SYSTEM_H

#include <stdint.h>

#include <Arduino.h>

namespace Frameduino
{
    struct pin_info_t;

    struct hal_system_interrupt_callback_t
    {
        uint8_t pin;
        void (*callback)(void *);
        void *user_data;
        hal_system_interrupt_callback_t *next;
    };

    struct hal_system_info_t
    {
        hal_system_interrupt_callback_t *interrupt_head;
    };

    inline hal_system_info_t *system_info = nullptr;

    namespace HAL
    {
        bool hal_register_interrupt_callback(pin_info_t *pin, void (*cb)(void *), void *user_data);
        bool hal_clear_interrupt_callback(pin_info_t *pin);

        void system_on_pin_interrupt(uint8_t reg, uint8_t pin);
    }

    inline void hal_system_enable()
    {
        system_info = new hal_system_info_t();
        Serial.println("Hal enabled");
    }

    inline void hal_system_tick() {}

}

#endif