#ifndef FRAMEDUINO_CORE_TIMER_H
#define FRAMEDUINO_CORE_TIMER_H

#include "core/timer.h"
namespace Frameduino
{

    class core_timer_t
    {
    public:
        core_timer_t() = default;
        core_timer_t(uint64_t interval)
        {
            this->interval = interval;
            start();
        };

        void start() { m_start_time = hal_millis() + interval; };
        bool has_triggered() const { return hal_millis() >= start_time; }

    private:
        uint64_t interval = 0;
        uint64_t start_time = 0xFFFFFFFFFFFFFFFF;
    };
}

#endif