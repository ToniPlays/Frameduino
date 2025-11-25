#include "spi_device.h"

#include <SPI.h>

namespace Frameduino
{

    spi_device_t::spi_device_t()
    {
        
    }

    spi_device_t::~spi_device_t()
    {
        hal_pin_detach(&cs_pin);
    }

    void spi_device_t::transaction(void (*cb)(spi_device_t *, void *), void* user_data)
    {
        hal_pin_write(&cs_pin, false);
        SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
        cb(this, user_data);
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