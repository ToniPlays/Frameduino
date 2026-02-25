#ifndef AVR_ATMEGA328P_PIN_MAPPING
#define AVR_ATMEGA328P_PIN_MAPPPING

#define PORT_B BIT(0)
#define PORT_C BIT(1)
#define PORT_D BIT(2)

#define PORT_COUNT 3

#define TIMER_0 BIT(0)
#define TIMER_1 BIT(1)
#define TIMER_2 BIT(2)
#define TIMER_COUNT 3

#define TIMER_CHANNEL_A BIT(0)
#define TIMER_CHANNEL_B BIT(1)


namespace Frameduino::HAL
{
    constexpr hardware_pin_t pin_map[] PROGMEM = {
        {0, PORT_D, PD0, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_PCINT},
        {1, PORT_D, PD1, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_PCINT},
        {2, PORT_D, PD2, PIN_CAP_IO | PIN_CAP_INTERRUPTS},
        {3, PORT_D, PD3, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_TIMER | PIN_CAP_INTERRUPTS},
        {4, PORT_D, PD4, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PCINT},
        {5, PORT_D, PD5, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT},
        {6, PORT_D, PD5, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {7, PORT_D, PD7, PIN_CAP_IO | PIN_CAP_PCINT},

        {8, PORT_B, PB0, PIN_CAP_IO | PIN_CAP_PCINT},
        {9, PORT_B, PB1, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {10, PORT_B, PB2, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_PCINT},
        {11, PORT_B, PB3, PIN_CAP_IO | PIN_CAP_PWM | PIN_CAP_TIMER | PIN_CAP_SPI | PIN_CAP_PCINT},
        {12, PORT_B, PB4, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_SPI},
        {13, PORT_B, PB5, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_SPI},

        {14, PORT_C, PC0, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {15, PORT_C, PC1, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {16, PORT_C, PC2, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {17, PORT_C, PC3, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC},
        {18, PORT_C, PC4, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_I2C | PIN_CAP_ADC},
        {19, PORT_C, PC5, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_I2C | PIN_CAP_ADC},
    };

    constexpr hardware_pwm_t pwm_pin_map[] PROGMEM = {
        {6, TIMER_0, TIMER_CHANNEL_A, 8},
        {5, TIMER_0, TIMER_CHANNEL_B, 8},
        {9, TIMER_1, TIMER_CHANNEL_A, 0},
        {10, TIMER_1, TIMER_CHANNEL_B, 0},
        {11, TIMER_2, TIMER_CHANNEL_A, 8},
        {3, TIMER_2, TIMER_CHANNEL_B, 8},
    };

    volatile uint8_t *get_port_from_index(uint8_t port_index)
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
    volatile uint8_t *get_input_register_for_port(volatile uint8_t *port)
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
    volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port)
    {
        if (port == &PORTB)
            return &DDRB;
        else if (port == &PORTC)
            return &DDRC;
        else if (port == &PORTD)
            return &DDRD;
        return nullptr;
    }

    const hardware_pin_t get_hardware_pin_from_pin(uint8_t pin)
    {
        hardware_pin_t tmp = {}; // temporary RAM copy
        for (uint8_t i = 0; i < ARRAY_SIZE(pin_map); i++)
        {
            memcpy_P(&tmp, &pin_map[i], sizeof(hardware_pin_t));
            if (tmp.pin_number == pin)
            {
                memcpy(&tmp, &tmp, sizeof(hardware_pin_t));
                return tmp;
            }
        }
        return tmp;
    }

    const hardware_pwm_t get_hardware_pwm_from_pin(pin_info_t *pin)
    {
        hardware_pwm_t tmp = {}; // temporary RAM copy
        for (uint8_t i = 0; i < ARRAY_SIZE(pin_map); i++)
        {
            memcpy_P(&tmp, &pwm_pin_map[i], sizeof(hardware_pwm_t));
            if (tmp.pin_number == pin->pin_number)
            {
                memcpy(&tmp, &tmp, sizeof(hardware_pwm_t));
                return tmp;
            }
        }
        return tmp;
    }

}

#endif