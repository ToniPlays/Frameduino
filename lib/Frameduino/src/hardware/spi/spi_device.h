#ifndef FRAMEDUINO_SPI_DEVICE_H
#define FRAMEDUINO_SPI_DEVICE_H

#include "hardware/gpio/pin.h"

namespace Frameduino
{
    class spi_device_t
    {
    public:
        spi_device_t(uint8_t pin);
        ~spi_device_t();

        bool transfer(uint8_t *bytes, uint8_t length);
        bool request(uint8_t *buffer, uint8_t length);

        void transaction(void (*cb)(spi_device_t *, void *), void* user_data);

        virtual void init() = 0;
        virtual void update() = 0;

    private:
        pin_info_t cs_pin;
    };
}

#endif