#include "hal_system.h"
#include "hardware/gpio/pin.h"

namespace Frameduino::HAL
{
    bool hal_register_interrupt_callback(pin_info_t *pin, void (*cb)(void *), void *user_data)
    {
        if (!system_info || !pin || !cb)
            return false;

        hal_system_interrupt_callback_t entry = {};
        entry.pin = pin->pin_number;
        entry.callback = cb;
        entry.user_data = user_data;
        system_info->interrupts.add(entry);

        hal_logger_log_i(("Enabled interrupt on pin: " + String(pin->pin_number)).c_str());
        return true;
    }

    bool hal_clear_interrupt_callback(pin_info_t *pin)
    {
        if (!system_info || !pin)
            return false;

        return system_info->interrupts.remove([](hal_system_interrupt_callback_t *m, void* pin) {
             return m->pin == ((pin_info_t*)pin)->pin_number;
        }, pin);
    }

    void system_on_pin_interrupt(uint8_t reg, uint8_t pin)
    {
        if (!system_info)
            return;

        system_info->interrupts.find([](hal_system_interrupt_callback_t *m, void* user_data) { 
            return m->pin == *(uint8_t*)user_data; 
        }, (void*)pin);
    }

    void system_register_pulse_on_pin(pin_info_t *pin, uint32_t expiration, bool end)
    {
        unsigned long now = millis();
        hal_logger_log_i(("Pulsing pin: " + String(pin->pin_number)).c_str());
        port_bit_write(pin->port, pin->mask, !end);
        for (int i = 0; i < 4; i++)
        {
            if (system_info->pulse_table[i].pin)
                continue;

            system_info->pulse_table[i].pin = pin;
            system_info->pulse_table[i].end_time = now + expiration;
            return;
        }
    }
    void system_pin_pulse_tick()
    {
        for (int i = 0; i < 4; i++)
        {
            if (system_info->pulse_table[i].pin && millis() >= system_info->pulse_table[i].end_time)
            {
                hal_pin_toggle(system_info->pulse_table[i].pin);
                system_info->pulse_table[i].pin = nullptr;
            }
        }
    }
}