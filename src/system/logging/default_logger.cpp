#include "default_logger.h"
#include "hardware/uart/hal_uart.h"
#include <Arduino.h>

namespace Frameduino
{
    hal_default_logger_t::hal_default_logger_t(uint32_t baud_rate)
    {
        hal_uart_enable(baud_rate);
    }

    hal_default_logger_t::~hal_default_logger_t()
    {

    }

    void hal_default_logger_t::log_v(const char* msg)
    {
        Serial.print(F("[V]: "));
        Serial.println(msg);
        Serial.flush();
    }
    void hal_default_logger_t::log_i(const char* msg)
    {
        Serial.print(F("[I]: "));
        Serial.println(msg);
        Serial.flush();
    }
    void hal_default_logger_t::log_w(const char* msg)
    {
        Serial.print(F("[W]: "));
        Serial.println(msg);
        Serial.flush();
    }
    void hal_default_logger_t::log_err(const char* msg)
    {
        Serial.print(F("[E]: "));
        Serial.println(msg);
        Serial.flush();
    }
}