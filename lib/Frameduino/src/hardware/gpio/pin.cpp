#include "pin.h"
#include "hardware_pin.h"
#include "hardware/timer/timer.h"

#include <string.h>
#include "Arduino.h"

namespace Frameduino
{
    bool hal_pin_attach(uint8_t pin, uint8_t usage, pin_info_t* info)
    {
        const HAL::hardware_pin_t p = HAL::get_hardware_pin_from_pin(pin);        

        if (!pin_supports_usage(p.capabilities, usage))
            return false;
            
        pin_info_t pin_info = {};
        pin_info.pin_number = p.pin_number;
        pin_info.mask = 1 << p.pin;
        pin_info.port = HAL::get_port_from_index(p.port);

        volatile uint8_t *reg = HAL::get_direction_register_for_port(pin_info.port);
        HAL::port_bit_write(reg, pin_info.mask, (usage & PIN_CONFIG_DIGITAL_OUTPUT));

        if(usage & PIN_CONFIG_PWM)
        {
            cli();
            //Enable proper timer
            const HAL::hardware_pwm_t pwm = HAL::get_hardware_pwm_from_pin(&pin_info);
            if(pwm.channel == -1) return false;
            
            HAL::enable_timer(pwm.timer);
            HAL::enable_timer_channels(pwm.timer, pwm.channel);
            sei();
        }

        if (info)
            memcpy(info, &pin_info, sizeof(pin_info_t));
        
        return true;
    }

    bool hal_pin_detach(pin_info_t info)
    {
        return false;
    }
}