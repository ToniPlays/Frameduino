#ifndef FRAMEDUINO_LOGGER_H
#define FRAMEDUINO_LOGGER_H

#include <stdint.h>
#include <Arduino.h>

namespace Frameduino
{
    class hal_logger_t
    {
    public:
        virtual void log_v(const char* msg) = 0;
        virtual void log_v(const String& msg) = 0;

        virtual void log_i(const char* msg) = 0;
        virtual void log_i(const String& msg) = 0;

        virtual void log_w(const char* msg) = 0;
        virtual void log_w(const String& msg) = 0;
        
        virtual void log_err(const char* msg) = 0;
        virtual void log_err(const String& msg) = 0;
    };
    
}

#endif