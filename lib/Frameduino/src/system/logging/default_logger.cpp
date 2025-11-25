#include "default_logger.h"
#include "hardware/uart/hal_uart.h"
#include <Arduino.h>

namespace Frameduino
{
    hal_default_logger_t::hal_default_logger_t()
    {
        hal_uart_enable(9600);
    }

    hal_default_logger_t::~hal_default_logger_t()
    {
    }

    void hal_default_logger_t::log_v(const char *msg)
    {
        Serial.print("[V]: ");
        Serial.println(msg);
    }
    void hal_default_logger_t::log_i(const char *msg)
    {
        Serial.print("[I]: ");
        Serial.println(msg);
    }
    void hal_default_logger_t::log_w(const char *msg)
    {
        Serial.print("[W]: ");
        Serial.println(msg);
    }
    void hal_default_logger_t::log_err(const char *msg)
    {
        Serial.print("[E]: ");
        Serial.println(msg);
    }
}