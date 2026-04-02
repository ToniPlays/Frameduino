#include "spi_device.h"

#include <SPI.h>

namespace Frameduino
{
    spi_device_t::spi_device_t(uint32_t clockSpeed) : m_ClockSpeed(clockSpeed)
    {
        
    }

    spi_device_t::~spi_device_t()
    {
        hal_pin_detach(&cs_pin);
    }

    void spi_device_t::start_transaction()
    {
        hal_pin_write(&cs_pin, false);
        SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
    }

    void spi_device_t::end_transaction()
    {
        SPI.endTransaction();
        hal_pin_write(&cs_pin, true);
    }

    bool spi_device_t::transfer(uint8_t *bytes, uint8_t length)
    {
        for(uint8_t i = 0; i < length; i++)
            SPI.transfer(bytes[i]);
        return true;
    }
    bool spi_device_t::request(uint8_t *buffer, uint8_t length)
    {
        for (uint8_t i = 0; i < length; i++)
            buffer[i] = SPI.transfer(0x00);
        return true;
    }
}