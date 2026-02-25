#ifndef AVR_ATTINY1634_PIN_MAPPING
#define AVR_ATTINY1634_PIN_MAPPPING

#define PORT_A BIT(0)
#define PORT_B BIT(1)
#define PORT_C BIT(2)

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
        {0,  PORT_B, PB0, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_ADC | PIN_CAP_PCINT},
        {1,  PORT_A, PA7, PIN_CAP_IO | PIN_CAP_UART | PIN_CAP_ADC | PIN_CAP_PCINT},
        {2,  PORT_A, PA6, PIN_CAP_IO | PIN_CAP_ADC | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT},
        {3,  PORT_A, PA5, PIN_CAP_IO | PIN_CAP_ADC | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT},
        {4,  PORT_A, PA4, PIN_CAP_IO | PIN_CAP_ADC | PIN_CAP_PCINT},
        {5,  PORT_A, PA3, PIN_CAP_IO | PIN_CAP_ADC | PIN_CAP_PCINT},
        {6,  PORT_A, PA2, PIN_CAP_IO | PIN_CAP_PCINT},
        {7,  PORT_A, PA1, PIN_CAP_IO | PIN_CAP_PCINT},
        {8,  PORT_A, PA0, PIN_CAP_IO | PIN_CAP_PCINT},
        {9,  PORT_C, PC5, PIN_CAP_IO | PIN_CAP_PCINT},
        {10, PORT_C, PC4, PIN_CAP_IO | PIN_CAP_PCINT},
        {11, PORT_C, PC2, PIN_CAP_IO | PIN_CAP_ADC | PIN_CAP_INTERRUPTS},
        {12, PORT_C, PC1, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_ADC | PIN_CAP_I2C | PIN_CAP_SPI},
        {13, PORT_C, PC0, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT | PIN_CAP_ADC},
        {14, PORT_B, PB3, PIN_CAP_IO | PIN_CAP_TIMER | PIN_CAP_PWM | PIN_CAP_PCINT | PIN_CAP_ADC},
        {15, PORT_B, PB2, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_UART | PIN_CAP_SPI},
        {16, PORT_B, PB1, PIN_CAP_IO | PIN_CAP_PCINT | PIN_CAP_UART | PIN_CAP_SPI | PIN_CAP_I2C},
        {17, PORT_C, PC3, PIN_CAP_IO | PIN_CAP_PCINT},       
    };

    constexpr hardware_pwm_t pwm_pin_map[] PROGMEM = {
        {2, TIMER_1, TIMER_CHANNEL_B, 8},
        {3, TIMER_0, TIMER_CHANNEL_B, 8},
        {13, TIMER_0, TIMER_CHANNEL_A, 8},
        {14, TIMER_1, TIMER_CHANNEL_A, 8}
    };

    volatile uint8_t *get_port_from_index(uint8_t port_index)
    {
        switch (port_index)
        {
        case PORT_A:
            return &PORTA;
        case PORT_B:
            return &PORTB;
        case PORT_C:
            return &PORTC;
        }
        return 0x00;
    }
    volatile uint8_t *get_input_register_for_port(volatile uint8_t *port)
    {
        if (port == &PORTA)
            return &PINA;
        else if (port == &PORTB)
            return &PINB;
        else if (port == &PORTC)
            return &PINC;
        else
            return nullptr;
    }
    volatile uint8_t *get_direction_register_for_port(volatile uint8_t *port)
    {
        if (port == &PORTA)
            return &DDRA;
        else if (port == &PORTB)
            return &DDRB;
        else if (port == &PORTC)
            return &DDRC;
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