#ifndef FRAMEDUINO_DEFAULT_LOGGER_H
#define FRAMEDUINO_DEFAULT_LOGGER_H

#include "logger.h"

namespace Frameduino
{
    class hal_default_logger_t : public hal_logger_t
    {
    public:
        hal_default_logger_t();
        ~hal_default_logger_t();

        void log_v(const char *msg) override;
        void log_i(const char *msg) override;
        void log_w(const char *msg) override;
        void log_err(const char *msg) override;
    };
}

#endif