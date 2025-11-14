#include "hal_system.h"
#include "hardware/gpio/pin.h"

namespace Frameduino::HAL
{
    bool hal_register_interrupt_callback(pin_info_t *pin, void (*cb)(void *), void *user_data)
    {
        if (!system_info || !pin || !cb)
            return false;

        // Allocate a new entry
        hal_system_interrupt_callback_t *entry = (hal_system_interrupt_callback_t *)malloc(sizeof(hal_system_interrupt_callback_t));
        if (!entry)
            return false;

        entry->pin = pin->pin_number;
        entry->callback = cb;
        entry->user_data = user_data;
        entry->next = nullptr;

        // Push onto the head of the linked list
        entry->next = system_info->interrupt_head;
        system_info->interrupt_head = entry;

        return true;
    }

    bool hal_clear_interrupt_callback(pin_info_t *pin)
    {
        if (!system_info || !pin)
            return false;

        hal_system_interrupt_callback_t *prev = nullptr;
        hal_system_interrupt_callback_t *curr = system_info->interrupt_head;

        while (curr)
        {
            if (curr->pin == pin->pin_number)
            {
                // Remove from list
                if (prev)
                    prev->next = curr->next;
                else
                    system_info->interrupt_head = curr->next;

                free(curr);
                return true;
            }
            prev = curr;
            curr = curr->next;
        }

        return false; // not found
    }

    void system_on_pin_interrupt(uint8_t reg, uint8_t pin)
    {
        if (!system_info)
            return;

        hal_system_interrupt_callback_t *curr = system_info->interrupt_head;
        while (curr)
        {
            if (curr->pin == pin && curr->callback)
                curr->callback(curr->user_data);
            curr = curr->next;
        }
    }

    void system_register_pulse_on_pin(pin_info_t *pin, uint32_t expiration, bool end)
    {
        unsigned long now = millis();
        HAL::port_bit_write(pin->port, pin->mask, !end);
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
        unsigned long now = millis();

        for (int i = 0; i < 4; i++)
        {
            if (system_info->pulse_table[i].pin && now >= system_info->pulse_table[i].end_time)
            {
                hal_pin_toggle(system_info->pulse_table[i].pin);
                system_info->pulse_table[i].pin = nullptr;
            }
        }
    }

}