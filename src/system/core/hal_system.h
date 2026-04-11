#ifndef FRAMEDUINO_HAL_SYSTEM_H
#define FRAMEDUINO_HAL_SYSTEM_H

#include <stdint.h>
#include "../logging/default_logger.h"
#include "boards/hal.h"
#include "linked_list.h"
#include "../memory/flash_string.h"
#include <Arduino.h>

namespace Frameduino
{
    struct pin_info_t;

    struct hal_pulse_t
    {
        pin_info_t *pin;
        uint32_t end_time;
    };

    struct hal_system_interrupt_callback_t
    {
        uint8_t pin;
        void (*callback)(void *);
        void *user_data;
    };

    class spi_device_t;

    struct hal_spi_device_t
    {
        uint8_t pin;
        spi_device_t *device;
    };

    struct hal_system_info_t
    {
        linked_list<hal_system_interrupt_callback_t> interrupts;
        linked_list<hal_spi_device_t> devices;
        hal_pulse_t pulse_table[4];
        hal_logger_t *logger = nullptr;
        void* user_data = nullptr;
        uint64_t millis = 0;
        uint64_t micros = 0;
    };

    extern hal_system_info_t *system_info;

    void hal_system_enable();
    void hal_logger_log_err_internal(const char* msg);

    namespace HAL
    {
        bool hal_register_interrupt_callback(pin_info_t *pin, void (*cb)(void *), void *user_data);
        bool hal_clear_interrupt_callback(pin_info_t *pin);

        void system_on_pin_interrupt(uint8_t reg, uint8_t pin);
        void system_register_pulse_on_pin(pin_info_t *pin, uint32_t expiration, bool end);

        void system_pin_pulse_tick();

        inline hal_system_info_t *hal_get_system_info()
        {
            if(!system_info) hal_logger_log_err_internal(F_STR("Get nullptr system info", 23).c_str());
            return system_info;
        }
    }

    void hal_logger_log_v(const char *msg);

    inline void hal_system_enable()
    {
        if (!system_info)
            system_info = new hal_system_info_t();
    }

    inline void* hal_system_get_user_data()
    {
        if (!system_info)
            return nullptr;
        return system_info->user_data;
    }

    inline void hal_system_set_user_data(void* data)
    {
        if (!system_info)
            system_info = new hal_system_info_t();
        system_info->user_data = data;
    }

    inline void hal_system_tick()
    {
        HAL::system_pin_pulse_tick();
    }

    inline void hal_logger_register(hal_logger_t *logger)
    {
        hal_system_enable();
        system_info->logger = logger;
    }
    
    inline void hal_logger_log_v_internal(const char* msg)
    {
        if (system_info->logger)
            system_info->logger->log_v(msg);
    }
    inline void hal_logger_log_v_internal(const String& msg)
    {
        if (system_info->logger)
            system_info->logger->log_v(msg.c_str());
    }
    inline void hal_logger_log_i_internal(const char* msg)
    {
        if (system_info->logger)
            system_info->logger->log_i(msg);
    }
    inline void hal_logger_log_i_internal(const String& msg)
    {
        if (system_info->logger)
            system_info->logger->log_i(msg.c_str());
    }
    inline void hal_logger_log_w_internal(const char* msg)
    {
        if (system_info->logger)
            system_info->logger->log_w(msg);
    }
    inline void hal_logger_log_w_internal(const String& msg)
    {
        if (system_info->logger)
            system_info->logger->log_w(msg.c_str());
    }

    inline void hal_logger_log_err_internal(const char* msg)
    {
        if (system_info->logger)
            system_info->logger->log_err(msg);
    }
    inline void hal_logger_log_err_internal(const String& msg)
    {
        if (system_info->logger)
            system_info->logger->log_err(msg.c_str());
    }
    
#ifdef FRAMEDUINO_DEBUG
    #define hal_logger_log_v(x) hal_logger_log_v_internal(x)
    #define hal_logger_log_i(x) hal_logger_log_i_internal(x)
    #define hal_logger_log_w(x) hal_logger_log_w_internal(x)
    #define hal_logger_log_err(x) hal_logger_log_err_internal(x)
#else 
    #define hal_logger_log_v(x)
    #define hal_logger_log_i(x)
    #define hal_logger_log_w(x)
    #define hal_logger_log_err(x)
#endif
}

#endif