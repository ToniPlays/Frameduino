#ifndef FRAMEDUINO_DIGITAL_PIN_H
#define FRAMEDUINO_DIGITAL_PIN_H

#include "Boards/Boards.h"

namespace Frameduino
{
    struct pin_info_t {
        volatile uint8_t* port;
        uint8_t index;
    };

    static pin_info_t hal_pin_attach(uint8_t pin, uint8_t usage);

    class DigitalPin : public Pin
    {

    public:
        DigitalPin() = default;
        DigitalPin(DigitalPinInfo info);

        ~DigitalPin();

        uint8_t Number() const override;
        volatile uint8_t *Port() const override;
        uint16_t GetConfig() const override;

        void Enable() override;
        void Disable() override;

        inline void Set(bool high)
        {
            HAL::port_bit_write(m_Port, m_Pin, high ? 1 : 0);
        }
        bool Read();
        inline void Toggle()
        {
            *m_Port ^= m_Pin;
        }
        void Pulse(uint32_t length);

    private:
        volatile uint8_t *m_Port = 0;
        uint8_t m_Pin = 0;
    };
}

#endif