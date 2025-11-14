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

}