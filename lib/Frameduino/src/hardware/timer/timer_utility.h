#ifndef FRAMEDUINO_TIMER_UTIL_H
#define FRAMEDUINO_TIMER_UTIL_H

namespace Frameduino::HAL
{
    struct timer_option_t
    {
        uint32_t freq;
        bool fast_pwm;
        uint8_t cs_bits;
    };

    timer_option_t choose_timer_config(uint32_t requestedFreq,
                                       const uint16_t prescalers[],
                                       const uint8_t bits[],
                                       size_t n)
    {
        timer_option_t options[n * 2];

        for (uint8_t i = 0; i < n; i++)
        {
            // Fast PWM: F_CPU / (prescaler × 256)
            options[i * 2] = {
                F_CPU / (prescalers[i] * 256UL),
                true,
                bits[i]};

            // Phase Correct PWM: F_CPU / (prescaler × 510)
            options[i * 2 + 1] = {
                F_CPU / (prescalers[i] * 510UL),
                false,
                bits[i]};
        }

        // Find closest frequency
        timer_option_t result = {0xFFFFFFFF, false, 0};
        uint32_t bestError = 0xFFFFFFFF;

        for (size_t i = 0; i < n * 2; i++)
        {
            uint32_t dist = (options[i].freq > requestedFreq) ? options[i].freq - requestedFreq : requestedFreq - options[i].freq;

            if (dist < bestError)
            {
                bestError = dist;
                result = options[i];
            }
        }

        return result;
    }
    struct timer_config_t
    {
        uint32_t actual_freq; // closest achievable frequency
        uint16_t ocr_value;   // OCR2A/B value
        uint8_t cs_bits;      // prescaler bits for TCCR2B
    };

    timer_config_t timer_compute_ocr(uint32_t requested_freq,
                                     const uint16_t prescalers[],
                                     const uint8_t bits[],
                                     size_t n,
                                     uint8_t resolution)
    {
        timer_config_t result = {0, 0, 0};
        uint32_t best_error = 0xFFFFFFFF;
        uint32_t limit = (1UL << resolution) - 1;

        for (size_t i = 0; i < n; i++)
        {
            uint64_t ocr64 = ((uint64_t)F_CPU) / (prescalers[i] * (uint64_t)requested_freq) - 1;

            if (ocr64 > limit)
                continue; // Doesn't fit, try next prescaler

            uint32_t ocr = (uint32_t)ocr64;
            uint32_t actual = F_CPU / (prescalers[i] * (ocr + 1));
            uint32_t error = (actual > requested_freq) ? actual - requested_freq : requested_freq - actual;

            if (error < best_error)
            {
                best_error = error;
                result.actual_freq = actual;
                result.ocr_value = ocr;
                result.cs_bits = bits[i];
            }
        }

        return result;
    }

}

#endif