#include "spi_device.h"

#include <SPI.h>

namespace Frameduino
{

    spi_device_t::spi_device_t(uint8_t pin)
    {
        hal_pin_attach(pin, PIN_CONFIG_DIGITAL_OUTPUT, &cs_pin);
        hal_pin_write(&cs_pin, true); // Write default high
    }

    spi_device_t::~spi_device_t()
    {
        hal_pin_detach(&cs_pin);
        SPI.end();
    }

    void spi_device_t::transaction(void (*cb)(spi_device_t *, void *), void* user_data)
    {
        SPI.beginTransaction(SPISettings());
        cb(this, user_data);
        SPI.endTransaction();
    }

    bool spi_device_t::transfer(uint8_t *bytes, uint8_t length)
    {
        SPI.transfer(bytes, length);
        return true;
    }
    bool spi_device_t::request(uint8_t *buffer, uint8_t length)
    {
        for (uint8_t i = 0; i < length; i++)
        {
            buffer[i] = SPI.transfer(0x00);
        }
        return true;
    }
}