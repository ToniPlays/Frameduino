#ifdef __AVR_ATmega328P__

#include "hardware/gpio/pin.h"
#include "hardware/timer/timer_utility.h"
#include "avr/io.h"

#include "ATmega328p_pin_map.h"
#include "ATmega328p_uart.h"
#include "ATmega328p_energy.h"

namespace Frameduino::HAL
{
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