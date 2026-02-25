#ifndef FRAMEDUINO_HAL_H
#define FRAMEDUINO_HAL_H

#include "hardware/gpio/hardware_pin.h"

namespace Frameduino
{
    struct pin_info_t;
}

namespace Frameduino::HAL
{
    //These are to be implemented by the platform
    volatile uint8_t *get_port_from_index(uint8_t port_index);
    volatile uint8_t *get_input_register_for_port(volatile uint8_t *port);
    volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port);

    const hardware_pin_t get_hardware_pin_from_pin(uint8_t pin);
    const hardware_pwm_t get_hardware_pwm_from_pin(pin_info_t *pin);

    bool attach_pin_interrupt(pin_info_t *pin);
    bool detach_pin_interrupt(pin_info_t *pin);

    bool pwm_write(pin_info_t *pin, uint16_t value);
    bool read_analog_pin(pin_info_t *pin, uint16_t *value);

    bool adc_enable();
    bool adc_set_conversion_time_prescaler(uint8_t prescaler);

    bool set_clock_prescaler(uint8_t prescaler);    //Ideally adjust all uart etc before switching

    void uart_begin(uint32_t baud, uint8_t config);
    bool uart_reset_baud_rate();


    void enable_sleep_mode(uint8_t mode);
}
#endif