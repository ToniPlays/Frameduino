#ifndef FRAMEDUINO_CORE_TIMER_H
#define FRAMEDUINO_CORE_TIMER_H

#include "core_timer.h"
#include "time.h"

namespace Frameduino
{

    class core_timer_t
    {
    public:
        core_timer_t() = default;
        core_timer_t(uint64_t interval)
        {
            this->m_Interval = interval;
            start();
        };

        void start() { m_Start_Time = hal_millis() + m_Interval; };
        bool has_triggered() const { return hal_millis() >= m_Start_Time; }

    private:
        uint64_t m_Interval = 0;
        uint64_t m_Start_Time = 0xFFFFFFFFFFFFFFFF;
    };
}

#endif