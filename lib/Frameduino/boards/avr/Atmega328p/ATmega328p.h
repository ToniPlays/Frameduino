#ifndef FRAMEDUINO_ARV_ATMEGA328P_H
#define FRAMEDUINO_ARV_ATMEGA328P_H

#include "Hardware/GPIO/HardwarePin.h"
#include "avr/io.h"

#define PORT_B BIT(0)
#define PORT_C BIT(1)
#define PORT_D BIT(2)

#define PORT_COUNT 3

namespace Frameduino::HAL
{

    constexpr HardwarePin pin_map[] = {
        {0, PORT_D, PD0, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_PCINT},
        {1, PORT_D, PD1, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_PCINT},
        {2, PORT_D, PD2, PIN_CAP_IO | PIN_CAP_INTERRUPTS},
        {3, PORT_D, PD3, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_INTERRUPTS},
        {4, PORT_D, PD4, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PCINT},
        {5, PORT_D, PD5, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT},
        {6, PORT_D, PD5, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {7, PORT_D, PD7, PIN_CAP_IO | PIN_CAP_PCINT},

        {8, PORT_B, PB0, PIN_CAP_IO | PIN_CAP_PCINT},
        {9, PORT_B, PB1, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {10, PORT_B, PB2, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {11, PORT_B, PB3, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_SPI | PIN_CAP_PCINT},
        {12, PORT_B, PB4, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_SPI},
        {13, PORT_B, PB5, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_SPI},

        {14, PORT_C, PC0, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {15, PORT_C, PC1, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {16, PORT_C, PC2, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {17, PORT_C, PC3, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {18, PORT_C, PC4, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_I2C | PIN_CAP_ADC},
        {19, PORT_C, PC5, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_I2C | PIN_CAP_ADC},
    };

    static volatile uint8_t *get_port_from_index(uint8_t port_index)
    {
        switch (port_index)
        {
        case PORT_B:
            return &PORTB;
        case PORT_C:
            return &PORTC;
        case PORT_D:
            return &PORTD;
        }
        return 0x00;
    }
    static volatile uint8_t *get_input_register_for_port(volatile uint8_t *port)
    {
        if (port == &PORTB)
            return &PINB;
        else if (port == &PORTC)
            return &PINC;
        else if (port == &PORTD)
            return &PIND;
        else
            return nullptr;
    }
    static volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port)
    {
        if (port == &PORTB)
            return &DDRB;
        else if (port == &PORTC)
            return &DDRC;
        else if (port == &PORTD)
            return &DDRD;
        else
            return nullptr;
    }

    static HardwarePin get_hardware_pin_from_pin(uint8_t pin)
    {
        for (uint8_t i = 0; i < ARRAY_SIZE(pin_map); i++)
            if (pin_map[i].PinNumber == pin)
                return pin_map[i];

        return HardwarePin(0, 0, 0, 0);
    }

}

#endif