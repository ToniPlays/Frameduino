#ifndef FRAMEDUINO_HAL_SYSTEM_H
#define FRAMEDUINO_HAL_SYSTEM_H

#include <stdint.h>
#include "../logging/default_logger.h"
#include <Arduino.h>

namespace Frameduino
{
    struct pin_info_t;

    struct hal_pulse_t
    {
        pin_info_t *pin;
        unsigned long end_time;
    };

    struct hal_system_interrupt_callback_t
    {
        uint8_t pin;
        void (*callback)(void *);
        void *user_data;
        hal_system_interrupt_callback_t *next;
    };

    struct hal_system_info_t
    {
        hal_system_interrupt_callback_t *interrupt_head;
        hal_pulse_t pulse_table[4];
        hal_logger_t *logger;
    };

    static hal_system_info_t *system_info = nullptr;

    namespace HAL
    {
        bool hal_register_interrupt_callback(pin_info_t *pin, void (*cb)(void *), void *user_data);
        bool hal_clear_interrupt_callback(pin_info_t *pin);

        void system_on_pin_interrupt(uint8_t reg, uint8_t pin);
        void system_register_pulse_on_pin(pin_info_t *pin, uint32_t expiration, bool end);

        void system_pin_pulse_tick();
    }

    void hal_logger_log_v(const char* msg);

    inline void hal_system_enable()
    {
        system_info = new hal_system_info_t();
        hal_logger_log_v("Hal system enabled");
    }

    inline void hal_system_tick()
    {
        HAL::system_pin_pulse_tick();
    }
    inline void hal_logger_register(hal_logger_t *logger)
    {
        system_info->logger = logger;
    }

    inline void hal_logger_log_v(const char *msg)
    {
        if(system_info->logger)
            system_info->logger->log_v(msg);
    }
    inline void hal_logger_log_i(const char *msg)
    {
        if(system_info->logger)
            system_info->logger->log_i(msg);
    }
    inline void hal_logger_log_w(const char *msg)
    {
        if(system_info->logger)
            system_info->logger->log_w(msg);
    }
    inline void hal_logger_log_err(const char *msg)
    {
        if(system_info->logger)
            system_info->logger->log_err(msg);
    }
}

#endif