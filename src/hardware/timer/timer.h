#ifndef FRAMEDUINO_TIMER_H
#define FRAMEDUINO_TIMER_H

#include "hardware/gpio/hardware_pin.h"

namespace Frameduino
{
    namespace HAL
    {
        bool enable_timer(uint8_t timer);
        bool enable_timer_channels(uint8_t timer, uint8_t channels);

        bool timer_set_frequency(uint8_t timer, uint32_t frequency);
        bool timer_set_resolution(uint8_t timer, uint8_t bits);

        bool timer_configure_interrupt(uint8_t timer, uint32_t frequency);
    }

    inline bool hal_timer_set_frequency(pin_info_t *pin, uint32_t frequency)
    {
        HAL::hardware_pwm_t pwm = HAL::get_hardware_pwm_from_pin(pin);
        if(pwm.channel == -1) return false;
        return HAL::timer_set_frequency(pwm.timer, frequency);
    }
    inline bool hal_timer_set_resolution(pin_info_t *pin, uint8_t bits)
    {
        return false;
    }
    //Only use OCxA
    inline bool hal_set_timer_interrupt(uint8_t timer, uint32_t frequency)
    {
        using namespace HAL;
        cli();
        bool result = HAL::timer_configure_interrupt(timer, frequency);
        sei();
        return result;
    }
}
#endif