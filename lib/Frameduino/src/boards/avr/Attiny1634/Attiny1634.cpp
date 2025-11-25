#ifdef __AVR_ATtiny1634__

#include "hardware/gpio/pin.h"
#include "hardware/timer/timer_utility.h"
#include "avr/io.h"

#define PORT_A BIT(0)
#define PORT_B BIT(1)
#define PORT_C BIT(2)

#define PORT_COUNT 3

#define TIMER_0 BIT(0)
#define TIMER_1 BIT(1)
#define TIMER_COUNT 2

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

    bool enable_timer(uint8_t timer)
    {
        switch (timer)
        {
        case TIMER_0:
            TCCR0A |= BIT(WGM01) | BIT(WGM00);
            TCCR0B |= BIT(CS00);
            break;
        case TIMER_1:
            TCCR1A |= BIT(WGM10);
            TCCR1B |= BIT(WGM12) | BIT(CS10);
            break;
        default:
            return false;
        }
        return true;
    }

    bool enable_timer_channels(uint8_t timer, uint8_t channels)
    {
        switch (timer)
        {
        case TIMER_0:
            if (channels & TIMER_CHANNEL_A)
                TCCR0A |= BIT(COM0A1);
            if (channels & TIMER_CHANNEL_B)
                TCCR0A |= BIT(COM0B1);
            break;
        case TIMER_1:
            if (channels & TIMER_CHANNEL_A)
                TCCR1A |= BIT(COM1A1);
            if (channels & TIMER_CHANNEL_B)
                TCCR1A |= BIT(COM1B1);
            break;
        default:
            return false;
        }
        return true;
    }

    bool timer_set_frequency(uint8_t timer, uint32_t frequency)
    {
        switch (timer)
        {
        case TIMER_0:
        {
            const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
            const uint8_t bits[] = {0b001, 0b010, 0b011, 0b100, 0b101};
            timer_option_t opt = choose_timer_config(frequency, prescalers, bits, 5);

            TCCR0A = (TCCR0A & ~((1 << WGM01) | (1 << WGM00))) | (!!opt.fast_pwm << WGM01) | (1 << WGM00);
            TCCR0B = (TCCR0B & ~((1 << WGM02) | 0b111)) | opt.cs_bits;

            return true;
        }
        default:
            return false; // Timer1 is 16-bit, use different method
        }

        return false;
    }
    bool timer_set_resolution(uint8_t timer, uint8_t bits)
    {
        return false;
    }

    bool timer_configure_interrupt(uint8_t timer, uint32_t frequency)
    {
        switch (timer)
        {
        case 0:
        {
            constexpr uint16_t prescalers[] = {1, 8, 64, 256, 1024};
            constexpr uint8_t cs_bits[] = {0b001, 0b010, 0b011, 0b100, 0b101};

            timer_config_t config = timer_compute_ocr(frequency, prescalers, cs_bits, 5, 16);
            TCCR0A = 0;
            TCNT0 = 0;
            OCR0A = config.ocr_value; // compare value for interrupt

            TCCR0B = BIT(WGM02);    // CTC mode
            TCCR0B |= config.cs_bits; // set prescaler

            TIMSK |= BIT(OCIE0A); // enable interrupt

            return true;
        }
        case 1:
        {
            constexpr uint16_t prescalers[] = {1, 8, 32, 64, 128, 256, 1024};
            constexpr uint8_t cs_bits[] = {0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111};

            timer_config_t config = timer_compute_ocr(frequency, prescalers, cs_bits, 7, 8);

            TCCR1A = 0;
            TCCR1B = 0;
            TCNT1 = 0;
            OCR1A = config.ocr_value; // compare value for interrupt

            TCCR1A = BIT(WGM11);   // CTC mode
            TCCR1B = config.cs_bits; // set prescaler
            TIMSK |= BIT(OCIE1A);   // enable interrupt

            return true;
        }
        default:
            return false; // Timer1 is 16-bit, use different method
        }

        return false;
    }

    bool pwm_write(pin_info_t *pin, uint16_t value)
    {
        const hardware_pwm_t pwm = get_hardware_pwm_from_pin(pin);
        if (pwm.channel == -1)
            return false;

        uint16_t scaled = value >> pwm.scalar;
        switch (pwm.timer)
        {
        case TIMER_0:
            switch (pwm.channel)
            {
            case TIMER_CHANNEL_A:
                OCR0A = scaled;
                break;
            case TIMER_CHANNEL_B:
                OCR0B = scaled;
                break;
            }
            break;
        case TIMER_1:
            switch (pwm.channel)
            {
            case TIMER_CHANNEL_A:
                OCR1A = scaled;
                break;
            case TIMER_CHANNEL_B:
                OCR1B = scaled;
                break;
            }
            break;
        }

        return true;
    }

    bool read_analog_pin(pin_info_t *pin, uint16_t *value)
    {
        uint8_t channel = pin->pin_number - 14;
        ADMUX = (ADMUX & 0xF0) | channel;
        ADCSRA |= BIT(ADSC);
        // Wait for conversion to finish
        while (ADCSRA & BIT(ADSC));

        // Read 10-bit result
        *value = ADC;
        return true;
    }

    bool adc_enable()
    {
        ADMUX = BIT(REFS0);
        return true;
    }
    
    bool adc_set_conversion_time_prescaler(uint8_t prescaler)
    {
        // Only ADPS[2:0] are allowed (0b000–0b111)
        if (prescaler > 7)
            return false;

        // Clear only the prescaler bits, preserve the rest (ADEN, ADSC, ADATE, ADIF, ADIE)
        ADCSRA = (ADCSRA & ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))) | (prescaler & 0x07);

        return true;
    }

    bool attach_pin_interrupt(pin_info_t *pin)
    {
        const hardware_pin_t hw_pin = get_hardware_pin_from_pin(pin->pin_number);
        switch (hw_pin.port)
        {
        case PORT_A:
            GIMSK |= BIT(PCIE0);
            PCMSK0 |= pin->mask;
            return true;
        case PORT_B:
            GIMSK |= BIT(PCIE1);
            PCMSK1 |= pin->mask;
            return true;
        case PORT_C:
            GIMSK |= BIT(PCIE2);
            PCMSK2 |= pin->mask;
            return true;
        }
        return false;
    }
    bool detach_pin_interrupt(pin_info_t *pin)
    {
        const hardware_pin_t hw_pin = get_hardware_pin_from_pin(pin->pin_number);
        switch (hw_pin.port)
        {
        case PORT_A:
            PCMSK0 = PCMSK0 & (~pin->mask);
            return true;
        case PORT_B:
            PCMSK1 = PCMSK1 & (~pin->mask);
            return true;
        case PORT_C:
            PCMSK2 = PCMSK2 & (~pin->mask);
            return true;
        }
        return false;
    }

    inline int pcint_to_arduino_pin(uint8_t pcint_num)
    {
        if (pcint_num <= 7)
            return 8 + pcint_num; // PORTB 0..7 → D8..D13
        else if (pcint_num <= 14)
            return 14 + (pcint_num - 8); // PORTC 0..6 → A0..A5
        else if (pcint_num <= 23)
            return pcint_num - 16; // PORTD 0..7 → D0..D7
        return -1;                 // invalid
    }

    ISR(PCINT0_vect)
    {
        static uint8_t prev = 0;
        uint8_t changed = (PINA ^ prev) & PCMSK0;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_A, pcint_to_arduino_pin(bit));

        prev = PINA;
    }

    ISR(PCINT1_vect)
    {
        static uint8_t prev = 0;
        uint8_t changed = (PINB ^ prev) & PCMSK1;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_B, pcint_to_arduino_pin(bit + 8));
        prev = PINB;
    }

    ISR(PCINT2_vect)
    {
        static uint8_t prev = 0;
        uint8_t changed = (PINC ^ prev) & PCMSK2;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_C, pcint_to_arduino_pin(bit + 16));

        prev = PINC;
    }

}

#endif