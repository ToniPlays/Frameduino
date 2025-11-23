#ifdef __AVR_ATmega328P__

#include "hardware/gpio/pin.h"
#include "hardware/timer/timer_utility.h"
#include "avr/io.h"

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

    bool enable_timer(uint8_t timer)
    {
        switch (timer)
        {
        case TIMER_0:
            TCCR0A |= BIT(WGM01) | BIT(WGM00);
            TCCR0B |= BIT(CS00);
            hal_logger_log_i("Enabled timer 0");
            break;
        case TIMER_1:
            TCCR1A |= BIT(WGM10);
            TCCR1B |= BIT(WGM12) | BIT(CS10);
            hal_logger_log_i("Enabled timer 1");
            break;
        case TIMER_2:
            TCCR2A |= BIT(WGM21) | BIT(WGM20);
            TCCR2B |= BIT(CS20);
            hal_logger_log_i("Enabled timer 2");
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
        case TIMER_2:
            if (channels & TIMER_CHANNEL_A)
                TCCR2A |= BIT(COM2A1);
            if (channels & TIMER_CHANNEL_B)
                TCCR2A |= BIT(COM2B1);
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

        case TIMER_2:
        {
            const uint16_t prescalers[] = {1, 8, 32, 64, 128, 256, 1024};
            const uint8_t bits[] = {0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111};

            timer_option_t opt = choose_timer_config(frequency, prescalers, bits, 7);
            TCCR2A = (TCCR2A & ~((1 << WGM21) | (1 << WGM20))) | (!!opt.fast_pwm << WGM21) | (1 << WGM20);

            TCCR2B = (TCCR2B & ~((1 << WGM22) | 0b111)) | opt.cs_bits;
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
        case 1:
        {
            constexpr uint16_t prescalers[] = {1, 8, 64, 256, 1024};
            constexpr uint8_t cs_bits[] = {0b001, 0b010, 0b011, 0b100, 0b101};

            timer_config_t config = timer_compute_ocr(frequency, prescalers, cs_bits, 5, 16);
            TCCR1A = 0;
            TCNT1 = 0;
            OCR1A = config.ocr_value; // compare value for interrupt

            TCCR1B = (1 << WGM12);    // CTC mode
            TCCR1B |= config.cs_bits; // set prescaler

            TIMSK1 |= BIT(OCIE1A); // enable interrupt
            hal_logger_log_i("Enabled timer interrupt 1");
            return true;
        }
        case 2:
        {
            constexpr uint16_t prescalers[] = {1, 8, 32, 64, 128, 256, 1024};
            constexpr uint8_t cs_bits[] = {0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111};

            timer_config_t config = timer_compute_ocr(frequency, prescalers, cs_bits, 7, 8);

            TCCR2A = 0;
            TCCR2B = 0;
            TCNT2 = 0;
            OCR2A = config.ocr_value; // compare value for interrupt

            TCCR2A = (1 << WGM21);   // CTC mode
            TCCR2B = config.cs_bits; // set prescaler
            TIMSK2 |= BIT(OCIE2A);   // enable interrupt

            hal_logger_log_i("Enabled timer interrupt 2");
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
        case TIMER_2:
            switch (pwm.channel)
            {
            case TIMER_CHANNEL_A:
                OCR2A = scaled;
                break;
            case TIMER_CHANNEL_B:
                OCR2B = scaled;
                break;
            }
            break;
        }

        return true;
    }

    bool attach_pin_interrupt(pin_info_t *pin)
    {
        const hardware_pin_t hw_pin = get_hardware_pin_from_pin(pin->pin_number);
        switch (hw_pin.port)
        {
        case PORT_B:
            PCICR |= BIT(PCIE0);
            PCMSK0 |= pin->mask;
            return true;
        case PORT_C:
            PCICR |= BIT(PCIE1);
            PCMSK1 |= pin->mask;
            return true;
        case PORT_D:
            PCICR |= BIT(PCIE2);
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
        case PORT_B:
            PCMSK0 = PCMSK0 & (~pin->mask);
            return true;
        case PORT_C:
            PCMSK1 = PCMSK1 & (~pin->mask);
            return true;
        case PORT_D:
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
        uint8_t changed = (PINB ^ prev) & PCMSK0;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_B, pcint_to_arduino_pin(bit));

        prev = PINB;
    }

    ISR(PCINT1_vect)
    {
        static uint8_t prev = 0;
        uint8_t changed = (PINC ^ prev) & PCMSK1;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_C, pcint_to_arduino_pin(bit + 8));
        prev = PINC;
    }

    ISR(PCINT2_vect)
    {
        static uint8_t prev = 0;
        uint8_t changed = (PIND ^ prev) & PCMSK2;

        for (uint8_t bit = 0; bit < 8; bit++)
            if (changed & (1 << bit))
                system_on_pin_interrupt(PORT_D, pcint_to_arduino_pin(bit + 16));

        prev = PIND;
    }

}

#endif