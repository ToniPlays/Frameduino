#include "DigitalPin.h"
#include "Boards/Boards.h"

#include <Arduino.h>

namespace Frameduino
{
    pin_info_t hal_pin_attach(uint8_t pin, uint8_t usage)
    {
        if(usage & PIN_CONFIG_ANALOG)
            return;

        HAL::HardwarePin p = HAL::get_hardware_pin_from_pin(info.Pin);
        m_Pin = 1 << p.Pin;
        m_Port = HAL::get_port_from_index(p.Port);
        
        volatile uint8_t* reg = HAL::get_direction_register_for_port(m_Port);
        HAL::port_bit_write(reg, m_Pin, (info.Config & PIN_CONFIG_DIGITAL_OUTPUT) != 0);
    }

    /*
    DigitalPin::~DigitalPin()
    {
        Disable();
    }

    uint8_t DigitalPin::Number() const
    {
        return m_Pin;
    }

    volatile uint8_t* DigitalPin::Port() const
    {
        return m_Port;
    }

    uint16_t DigitalPin::GetConfig() const
    {
        return 0;
    }

    void DigitalPin::Enable()
    {

    }

    void DigitalPin::Disable()
    {

    }

    bool DigitalPin::Read()
    {
        uint8_t ddr = *HAL::get_direction_register_for_port(m_Port);
        if(ddr & m_Pin)
            return HAL::port_bit_read(m_Port, m_Pin);
        return HAL::port_bit_read_input(m_Port, m_Pin);
    }
    
    void DigitalPin::Pulse(uint32_t length)
    {
        Set(true);
        delay(length);
        Set(false);
    }
    */
}